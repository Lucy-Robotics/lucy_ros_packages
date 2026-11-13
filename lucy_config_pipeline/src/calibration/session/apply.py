"""Building and writing the calibration preset from the accepted joints."""

from __future__ import annotations

import json

from lucy_config_generator.calibration import apply_to_text
from lucy_config_generator.calibration import CalibrationError
from lucy_config_generator.calibration import homing_offsets_by_actuator
from lucy_config_generator.calibration import set_calibration_block
from lucy_config_generator.calibration import urdf_limits_from_config
from lucy_config_generator.calibration import validate_calibration
from lucy_config_generator.schema import validate_hardware_yaml
import yaml

from .constants import APPLIED
from .constants import MARGIN_RAD
from .constants import PRESET_NAME
from .constants import REVIEW
from .constants import SessionError
from .constants import TEMPLATE_PRESET
from ...config_store.io import atomic_write_text


class SessionApplyMixin:
    """Turn the accepted joints into a hardware preset, then write it out."""

    def _apply(self, argument: str) -> None:
        self._need_phase(REVIEW)
        name = argument.strip() or f"{self._prefix}_{self._serial}"
        if not PRESET_NAME.fullmatch(name) or name == TEMPLATE_PRESET:
            raise SessionError("bad_preset_name", f"Not a usable preset name: {name}")

        calibration = self.lerobot_calibration()
        try:
            validate_calibration(calibration)
            text, _limits, _warnings = apply_to_text(
                self._store.read_named_yaml(TEMPLATE_PRESET),
                calibration,
                margin_rad=MARGIN_RAD,
                full_turn_motors=[
                    j["motor"] for j in self._joints if j.get("full_turn")
                ],
            )
            config = yaml.safe_load(text)
            text = set_calibration_block(
                text,
                calibrated_at=self._clock(),
                homing_offsets=homing_offsets_by_actuator(calibration, config),
                adapter_serial=self._serial,
            )
            config = yaml.safe_load(text)
            validate_hardware_yaml(config)
            urdf_limits_from_config(config)
        except (CalibrationError, KeyError, OSError, ValueError, yaml.YAMLError) as e:
            raise SessionError(
                "apply_failed", f"Could not build the preset: {e}"
            ) from e

        self._store.write_named_yaml(name, text)
        atomic_write_text(
            self._store.lerobot_json_path(name),
            json.dumps(calibration, indent=4) + "\n",
        )
        self._preset = name
        self._phase = APPLIED
        self._journal()
