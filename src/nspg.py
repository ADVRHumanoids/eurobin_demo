from cartesio_planning import NSPG
from cartesio_planning.planning import PositionCartesianSolver
import cartesio_planning.validity_check as vc

from cartesian_interface.pyci_all import *

import yaml

class CentauroNSPG:
    def __init__(self, model, dynamic_links, static_links):
        ik_str = self._generate_ik_cfg(dynamic_links + static_links)
        self.ci = pyci.CartesianInterface.MakeInstance('OpenSot', ik_str, model, 1.)
        self.ik = PositionCartesianSolver(self.ci)
        self.model = model
        self.vc = self._make_vc_context()
        self._nspg = NSPG.NSPG(ik_solver=self.ik, vc_context=self.vc)

    def set_validity_checker(self, vc):
        self.vc = vc
        self._nspg = NSPG.NSPG(ik_solver=self.ik, vc_context=self.vc)

    def set_references(self, links, poses):
        [self.ik.setDesiredPose(link, pose) for link, pose in zip(links, poses)]

    def sample(self, timeout):
        return self._nspg.sample(timeout)

    def _generate_ik_cfg(self, links):
        # write cartesio config
        cfg = dict()

        cfg['solver_options'] = {'regularization': 1e-2}

        cfg['stack'] = [
            list(links), ['postural']  # discard postural
        ]

        cfg['constraints'] = ['joint_limits']

        cfg['joint_limits'] = {
            'type': 'JointLimits'
        }

        cfg['postural'] = {
            'name': 'postural',
            'type': 'Postural',
            'lambda': 0.1
        }

        for c in links:
            cfg[c] = {
                'type': 'Cartesian',
                'indices': [0, 1, 2],
                'distal_link': c
            }

        return yaml.dump(cfg)

    def _make_vc_context(self):
        _planner_config = dict()
        _planner_config['state_validity_check'] = ['collisions']
        _planner_config['collisions'] = {'type': 'CollisionCheck', 'include_environment': 'true'}

        vc_context = vc.ValidityCheckContext(yaml.dump(_planner_config), self.model)
        return vc_context