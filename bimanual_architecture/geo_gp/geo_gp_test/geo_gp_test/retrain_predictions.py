# flake8: noqa
"""Retrain reference GP skills and replace saved prediction artifacts.

The script rebuilds each referenced skill from ``refs/processed`` with the
current Geo-GP code and ``config/default.yaml``. It deliberately does not load,
save, or modify existing ``.pt`` model files.
"""

import argparse
import csv
import json
import logging
import os
import random
import re
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from geometry_msgs.msg import Pose, Vector3
from geo_gp_interfaces.msg import PromptTrajectory

from geo_gp_prediction.predictor import (
    Predictor,
    save_predicted_trajectory_to_csv,
    save_similarity_transform_to_json,
)

# predictor.py adds the standard /home/user/geo-gp checkout to sys.path. These
# imports therefore use the same current implementation as the online node.
from config import Config
from skills.skill_library import SkillLibrary
from skills.skill_loader import load_skill_csv


DEFAULT_GEO_GP_ROOT = Path('/home/user/geo-gp')
DEFAULT_PRED_ROOTS = (
    DEFAULT_GEO_GP_ROOT / 'data/06-02/preds',
    DEFAULT_GEO_GP_ROOT / 'data/06-03/preds',
)
ARTIFACT_RE = re.compile(
    r'^(?P<kind>prompt|prediction|similarity_transform)_'
    r'(?P<status>success|failed|fail)_'
    r'(?P<stamp>\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})\.(?P<ext>csv|json)$'
)


@dataclass(frozen=True)
class Trial:
    """Files belonging to one timestamped prediction attempt."""

    directory: Path
    stamp: str
    prompt_path: Path
    prediction_path: Path
    alignment_path: Path
    reference_name: str
    reference_path: Path


def parse_args(argv=None):
    """Parse command-line options."""
    parser = argparse.ArgumentParser(
        description=(
            'Retrain the reference named by each saved similarity transform, '
            'roll out its prompt, and replace prediction/alignment artifacts.'
        )
    )
    parser.add_argument(
        '--pred-roots',
        nargs='+',
        type=Path,
        default=list(DEFAULT_PRED_ROOTS),
        help=(
            'Prediction roots. Each root is expected beside refs/processed '
            '(default: 06-02/preds and 06-03/preds).'
        ),
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=DEFAULT_GEO_GP_ROOT / 'config/default.yaml',
        help='Current Geo-GP YAML configuration.',
    )
    parser.add_argument(
        '--predict-force',
        choices=('auto', 'on', 'off'),
        default='auto',
        help=(
            'Force rollout mode. auto preserves the old prediction schema: '
            'force is enabled only when its CSV has fx/fy/fz columns.'
        ),
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Only discover and validate trial/reference pairings; write nothing.',
    )
    parser.add_argument(
        '--limit',
        type=int,
        default=None,
        help='Process at most this many trials (useful for a trial run).',
    )
    parser.add_argument(
        '--fail-fast',
        action='store_true',
        help='Stop at the first invalid or failed trial instead of continuing.',
    )
    return parser.parse_args(argv)


def artifact_identity(path):
    """Return the parsed artifact identity or raise a descriptive error."""
    match = ARTIFACT_RE.match(path.name)
    if match is None:
        raise ValueError(f'unrecognized artifact filename: {path}')
    return match.groupdict()


def unique_companion(directory, kind, stamp, extension):
    """Find exactly one status variant of a companion artifact."""
    matches = []
    for path in directory.glob(f'{kind}_*_{stamp}.{extension}'):
        try:
            identity = artifact_identity(path)
        except ValueError:
            continue
        if identity['kind'] == kind and identity['stamp'] == stamp:
            matches.append(path)
    if len(matches) != 1:
        raise ValueError(
            f'expected one {kind} artifact for {directory}/{stamp}, found {matches}'
        )
    return matches[0]


def load_reference_name(alignment_path):
    """Read the exact reference/skill name saved by the old alignment."""
    with alignment_path.open('r', encoding='utf-8') as stream:
        payload = json.load(stream)
    name = str(payload.get('skill_name', '')).strip()
    if not name:
        raise ValueError(f'{alignment_path} has no non-empty skill_name')
    if Path(name).name != name:
        raise ValueError(f'unsafe skill_name in {alignment_path}: {name!r}')
    return name


def discover_trials(pred_roots):
    """Discover complete prompt/prediction/alignment triples."""
    trials = []
    seen = set()
    for raw_root in pred_roots:
        pred_root = raw_root.expanduser().resolve()
        if not pred_root.is_dir():
            raise FileNotFoundError(f'prediction root does not exist: {pred_root}')
        reference_root = pred_root.parent / 'refs' / 'processed'
        if not reference_root.is_dir():
            raise FileNotFoundError(f'reference root does not exist: {reference_root}')

        for alignment_path in sorted(pred_root.rglob('similarity_transform_*.json')):
            identity = artifact_identity(alignment_path)
            key = (alignment_path.parent, identity['stamp'])
            if key in seen:
                raise ValueError(f'duplicate alignment artifacts for {key}')
            seen.add(key)
            prompt_path = unique_companion(
                alignment_path.parent, 'prompt', identity['stamp'], 'csv'
            )
            prediction_path = unique_companion(
                alignment_path.parent, 'prediction', identity['stamp'], 'csv'
            )
            reference_name = load_reference_name(alignment_path)
            reference_path = reference_root / f'{reference_name}.csv'
            if not reference_path.is_file():
                raise FileNotFoundError(
                    f'reference {reference_name!r} from {alignment_path} not found at '
                    f'{reference_path}'
                )
            trials.append(
                Trial(
                    directory=alignment_path.parent,
                    stamp=identity['stamp'],
                    prompt_path=prompt_path,
                    prediction_path=prediction_path,
                    alignment_path=alignment_path,
                    reference_name=reference_name,
                    reference_path=reference_path,
                )
            )
    return sorted(trials, key=lambda trial: (str(trial.directory), trial.stamp))


def read_prompt(path):
    """Read a saved prompt CSV as a PromptTrajectory message."""
    prompt = PromptTrajectory()
    required = ('time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
    with path.open('r', newline='') as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        missing = [field for field in required if field not in fields]
        if missing:
            raise ValueError(f'{path} is missing columns: {missing}')
        has_force = all(field in fields for field in ('fx', 'fy', 'fz'))
        for row in reader:
            pose = Pose()
            pose.position.x = float(row['x'])
            pose.position.y = float(row['y'])
            pose.position.z = float(row['z'])
            pose.orientation.x = float(row['qx'])
            pose.orientation.y = float(row['qy'])
            pose.orientation.z = float(row['qz'])
            pose.orientation.w = float(row['qw'])
            prompt.poses.append(pose)
            prompt.time_from_start.append(float(row['time']))
            if has_force:
                force = Vector3()
                force.x = float(row['fx'])
                force.y = float(row['fy'])
                force.z = float(row['fz'])
                prompt.forces.append(force)
    if not prompt.poses:
        raise ValueError(f'{path} contains no prompt samples')
    prompt.header.frame_id = 'world'
    return prompt


def prediction_has_force(path):
    """Check whether the old prediction CSV was produced in force mode."""
    with path.open('r', newline='') as stream:
        fields = csv.DictReader(stream).fieldnames or []
    return all(field in fields for field in ('fx', 'fy', 'fz'))


def force_enabled(mode, prediction_path):
    """Resolve the requested force rollout mode for one trial."""
    if mode == 'on':
        return True
    if mode == 'off':
        return False
    return prediction_has_force(prediction_path)


def set_training_seed(config):
    """Reset random generators so each reference is reproducibly trained."""
    seed = int(config.experiment.get('seed', 0))
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def train_reference(reference_path, config):
    """Train one in-memory 6D skill with current code and YAML parameters."""
    set_training_seed(config)
    skill = load_skill_csv(str(reference_path), mode='6d')
    skill.smooth_win = int(config.sampling['smooth_win'])
    skill.train_gp(
        k=int(config.sampling['k_hist']),
        input_type='spherical',
        output_type='delta',
        train_ratio=float(config.dataset['train_ratio']),
    )
    return skill


def select_only_skill(predictor, skill):
    """Force matching against the reference named by the saved alignment."""
    library = SkillLibrary()
    library.add_skill(skill)
    predictor.skill_library = library


def status_path(path, status):
    """Return an artifact path with only its success/failure token changed."""
    identity = artifact_identity(path)
    filename = (
        f"{identity['kind']}_{status}_{identity['stamp']}.{identity['ext']}"
    )
    return path.with_name(filename)


def temporary_output_path(destination):
    """Build a same-directory temporary path for an atomic replacement."""
    return destination.with_name(f'.{destination.name}.tmp-{os.getpid()}')


def replace_artifacts(trial, predicted, context):
    """Replace prediction and alignment, then normalize their status names."""
    if not context or not context.get('ok'):
        raise RuntimeError(
            f'prediction produced no reusable alignment context for {trial.prompt_path}'
        )

    status = 'success' if predicted.success else 'fail'
    prediction_target = status_path(trial.prediction_path, status)
    alignment_target = status_path(trial.alignment_path, status)
    prediction_tmp = temporary_output_path(prediction_target)
    alignment_tmp = temporary_output_path(alignment_target)

    try:
        save_predicted_trajectory_to_csv(str(prediction_tmp), predicted)
        save_similarity_transform_to_json(str(alignment_tmp), context)
        os.replace(prediction_tmp, prediction_target)
        os.replace(alignment_tmp, alignment_target)
    finally:
        prediction_tmp.unlink(missing_ok=True)
        alignment_tmp.unlink(missing_ok=True)

    if trial.prediction_path != prediction_target:
        trial.prediction_path.unlink(missing_ok=True)
    if trial.alignment_path != alignment_target:
        trial.alignment_path.unlink(missing_ok=True)
    return prediction_target, alignment_target


def create_predictor(logger, config_path, empty_model_dir):
    """Create a Predictor without loading any saved model files."""
    return Predictor(logger, str(config_path), str(empty_model_dir))


def run(args):
    """Execute discovery, in-memory training, rollout, and replacement."""
    logger = logging.getLogger('retrain_predictions')
    config_path = args.config.expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f'config does not exist: {config_path}')
    expected_config = (DEFAULT_GEO_GP_ROOT / 'config/default.yaml').resolve()
    if config_path != expected_config:
        logger.warning(
            'Skill training reads %s through config.runtime; Predictor was given %s. '
            'Use current default.yaml unless both resolve to the same configuration.',
            expected_config,
            config_path,
        )

    trials = discover_trials(args.pred_roots)
    if args.limit is not None:
        if args.limit < 0:
            raise ValueError('--limit must be non-negative')
        trials = trials[:args.limit]
    logger.info('Discovered %d complete trials', len(trials))
    for trial in trials:
        logger.info(
            'PAIR trial=%s stamp=%s reference=%s',
            trial.directory,
            trial.stamp,
            trial.reference_path,
        )
    if args.dry_run:
        return 0
    if not trials:
        raise RuntimeError('no complete trials found')

    config = Config(str(config_path))
    trained_skills = {}
    succeeded = 0
    prediction_failures = 0
    errors = 0

    with tempfile.TemporaryDirectory(prefix='geo_gp_retrain_empty_models_') as empty_dir:
        predictor = create_predictor(logger, config_path, Path(empty_dir))
        for index, trial in enumerate(trials, start=1):
            logger.info(
                '[%d/%d] Rebuilding %s using %s',
                index,
                len(trials),
                trial.directory,
                trial.reference_name,
            )
            try:
                skill = trained_skills.get(trial.reference_path)
                if skill is None:
                    skill = train_reference(trial.reference_path, config)
                    trained_skills[trial.reference_path] = skill
                select_only_skill(predictor, skill)
                prompt = read_prompt(trial.prompt_path)
                predict_force = force_enabled(args.predict_force, trial.prediction_path)
                predicted = predictor.predict(prompt, predict_force=predict_force)
                prediction_path, alignment_path = replace_artifacts(
                    trial, predicted, predictor.last_prediction_context
                )
                if predicted.success:
                    succeeded += 1
                else:
                    prediction_failures += 1
                logger.info(
                    '[%d/%d] WROTE success=%s poses=%d force=%s prediction=%s '
                    'alignment=%s',
                    index,
                    len(trials),
                    predicted.success,
                    len(predicted.poses),
                    predict_force,
                    prediction_path,
                    alignment_path,
                )
            except Exception:
                errors += 1
                logger.exception('[%d/%d] ERROR trial=%s', index, len(trials), trial.directory)
                if args.fail_fast:
                    raise

    logger.info(
        'Finished: success=%d prediction_fail=%d errors=%d trained_references=%d',
        succeeded,
        prediction_failures,
        errors,
        len(trained_skills),
    )
    return 1 if errors else 0


def main(argv=None):
    """Command-line entry point."""
    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')
    try:
        return run(parse_args(argv))
    except Exception:
        logging.getLogger('retrain_predictions').exception('Fatal error')
        return 1


if __name__ == '__main__':
    sys.exit(main())
