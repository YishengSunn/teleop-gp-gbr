"""Build current CSV-backed reference predictors for offline evaluation."""

from pathlib import Path
import tempfile

from config import Config
from geo_gp_prediction.predictor import Predictor
import numpy as np
from skills.skill_loader import load_skills_from_folder
import torch


def resolve_reference_dir(input_path, configured_path=''):
    """Resolve ``refs/processed`` beside the input dataset's ``preds`` root."""
    if configured_path:
        return Path(configured_path).expanduser().resolve()

    input_path = Path(input_path).expanduser().resolve()
    candidate = input_path if input_path.is_dir() else input_path.parent
    for ancestor in (candidate, *candidate.parents):
        if ancestor.name == 'preds':
            return ancestor.parent / 'refs' / 'processed'
    raise FileNotFoundError(
        f'cannot infer refs/processed from input_path={input_path}; '
        'set the reference_dir parameter explicitly'
    )


def make_current_reference_predictor(
        node, input_path, config_path, configured_reference_dir=''):
    """Load current reference CSVs without reading any serialized GP model."""
    reference_dir = resolve_reference_dir(input_path, configured_reference_dir)
    if not reference_dir.is_dir():
        raise FileNotFoundError(
            f'reference_dir does not exist: {reference_dir}'
        )

    training_config = Config(config_path)
    model_tempdir = tempfile.TemporaryDirectory(
        prefix='offline_fusion_empty_models_'
    )
    predictor = Predictor(node.get_logger(), config_path, model_tempdir.name)
    skills = load_skills_from_folder(str(reference_dir), mode='6d')
    if not skills:
        model_tempdir.cleanup()
        raise FileNotFoundError(
            f'no reference CSV files found in {reference_dir}'
        )

    for skill in skills:
        skill.smooth_win = int(training_config.sampling['smooth_win'])
        skill.prepare_reference()
        predictor.skill_library.add_skill(skill)

    node._reference_training_config = training_config
    node._reference_model_tempdir = model_tempdir
    node.get_logger().info(
        f'Loaded {len(skills)} current references from {reference_dir}; '
        'GP models will be trained on first use'
    )
    return predictor


def ensure_current_skill_trained(node, skill):
    """Train a current CSV-backed reference skill once, in memory."""
    if skill.model is not None:
        return
    config = getattr(node, '_reference_training_config', None)
    if config is None:
        raise RuntimeError('current reference training config is unavailable')

    seed = int(config.experiment.get('seed', 0))
    np.random.seed(seed)
    torch.manual_seed(seed)
    node.get_logger().info(
        f'Training current GP model for reference {skill.name}'
    )
    skill.train_gp(
        k=int(config.sampling['k_hist']),
        input_type='spherical',
        output_type='delta',
        train_ratio=float(config.dataset['train_ratio']),
    )
