import yaml
from cartesian_interface.pyci_all import *
from cartesio_planning import constraints

def make_constraint(model, ctrl_points):

    # write cartesio config
    cs_cfg = dict()

    cs_cfg['solver_options'] = {'regularization': 1e-2}

    cs_cfg['stack'] = [
        [c for c in ctrl_points if c != ctrl_points]  # discard postural
    ]

    cs_cfg['constraints'] = ['joint_limits']

    cs_cfg['joint_limits'] = {
        'type': 'JointLimits',
    }

    for c in ctrl_points:
        cs_cfg[c] = {
            'type': 'Cartesian',
            'indices': [0, 1, 2],
            'distal_link': c
        }

    cs_str = yaml.dump(cs_cfg)

    cs_ci = pyci.CartesianInterface.MakeInstance('OpenSot', cs_str, model, 0.01)

    return constraints.CartesianConstraint(cs_ci)