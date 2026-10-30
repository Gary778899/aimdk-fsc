"""Current customer-interface compensation; no per-robot calibration lookup.

These are fixed operating parameters, not a measured calibration for every robot.
Legacy engineering/calibration tools retain their separate configuration paths.
"""


def fixed_compensation() -> dict:
    """用途：返回上游固定补偿参数的独立副本。 返回计算结果，具体结构见下方原始说明或实现。
Return an independent copy of the MDI / public MoveJ parameters."""
    return dict(stiffness=40.0, bias_limit_deg=12.0, gravity_source="pelvis")
