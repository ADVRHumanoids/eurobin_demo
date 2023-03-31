from cartesio_planning import planning
from cartesio_planning import validity_check
from cartesio_planning import visual_tools

from cartesian_interface.pyci_all import *

from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co

import nspg
import manifold

import numpy as np
import scipy.linalg as la
import scipy.interpolate as interpol
import yaml
import rospy

class Planner:
    def __init__(self, urdf, srdf):
        opt = co.ConfigOptions()
        opt.set_urdf(urdf)
        opt.set_srdf(srdf)
        opt.generate_jidmap()
        opt.set_bool_parameter('is_model_floating_base', True)
        opt.set_string_parameter('model_type', 'RBDL')
        self.model = xbot.ModelInterface(opt)

        qhome = self.model.getRobotState('home')
        self.model.setJointPosition(qhome)
        self.model.update()

        self.rspub = pyci.RobotStatePublisher(self.model)

        self.start_viz = visual_tools.RobotViz(self.model,
                                               '/centauro/start',
                                               color=[0, 0, 1, 0.5],
                                               tf_prefix='ci/')

        self.goal_viz = visual_tools.RobotViz(self.model,
                                              '/centauro/goal',
                                              color=[0, 1, 0, 0.5],
                                              tf_prefix='ci/')

        self.static_links = ['contact_1', 'contact_2', 'contact_3', 'contact_4']
        self.dynamic_links = ['arm1_8', 'arm2_8']
        self.nspg = nspg.CentauroNSPG(self.model, self.dynamic_links, self.static_links)

        self.nspg.vc.planning_scene.addBox('anymal', [0.4, 0.6, 0.6], Affine3([1.0, 0.0, -0.4], [0., 0., 0., 1.]))
        self.nspg.vc.planning_scene.addBox('parcel', [0.2, 0.3, 0.1], Affine3([1.0, 0.0, -0.05], [0., 0., 0., 1.]))
        self.nspg.vc.planning_scene.startGetPlanningSceneServer()

        # joint limits for the planner
        qmin, qmax = self.model.getJointLimits()

        qmin = np.full(self.model.getJointNum(), -6.0)
        qmax = -qmin


        self.qmin = qmin
        self.qmax = qmax

        # define start and goal configurations
        self.qstart = qhome
        self.qgoal = []

    def generate_start_pose(self):
        self.model.setJointPosition(self.qstart)
        self.model.update()
        self.start_viz.publishMarkers([])

    def generate_goal_pose(self, links, poses):
        self.nspg.set_references(links, poses)
        success = self.nspg.sample(5.)
        self.qgoal = self.model.getJointPosition()
        self.goal_viz.publishMarkers([])
        if not success:
            raise Exception('unable to find a goal configuration!')



    def plan(self, planner_type='RRTConnect', timeout=1.0, threshold=0.0):

        # manifold
        constr = manifold.make_constraint(self.model, self.static_links)

        planner_config = {
            'state_space': {'type': 'Atlas'}
        }

        # create planner
        planner = planning.OmplPlanner(
            constr,
            self.qmin, self.qmax,
            yaml.dump(planner_config)
        )

        ### TODO: add check for start and goal state w.r.t. the manifold
        planner.setStartAndGoalStates(self.qstart, self.qgoal, threshold)

        def validity_predicate(q):
            self.model.setJointPosition(q)
            self.model.update()
            return self.nspg.vc.checkAll()

        planner.setStateValidityPredicate(validity_predicate)
        success = planner.solve(timeout, planner_type)

        print('Planner output : {}'.format(success))

        if success:
            solution = np.array(planner.getSolutionPath()).transpose()
            error = solution[:, -1] - np.array(self.qgoal)
            print('Error is {} rad'.format(la.norm(error)))
            return solution
        else:
            return None

    def play_on_rviz(self, solution, ntimes, duration):

        # play solution a number of times...
        dt = duration / solution.shape[1]

        for _ in range(ntimes):
            for i in range(solution.shape[1]):
                q = solution[:, i]
                self.model.setJointPosition(q)
                self.model.update()
                self.rspub.publishTransforms('ci')
                rospy.sleep(dt)

    def interpolate(self, solution, dt, max_qdot, max_qddot):

        qsize = solution.shape[0]
        nknots = solution.shape[1]

        seg_durs = []
        seg_vels = []

        safety_factor = 1.2

        for i in range(nknots - 1):
            tk = np.abs(((solution[:, i + 1] - solution[:, i]) / max_qdot)).max() * safety_factor
            tk = max(tk, 0.001)
            seg_vels.append((solution[:, i + 1] - solution[:, i]) / tk)
            seg_durs.append(tk)

        print(seg_durs)

        for i in range(len(seg_vels) - 1):
            acc_max = np.abs((seg_vels[i + 1] - seg_vels[i]) / seg_durs[i]).max()
            if acc_max > max_qddot:
                seg_durs[i] *= acc_max / max_qddot * safety_factor

        print(seg_durs)

        seg_durs.insert(0, 0)

        times = np.cumsum(seg_durs)

        interpolators = []
        for i in range(qsize):
            inter = interpol.UnivariateSpline(times, solution[i, :], s=0)
            interpolators.append(inter)

        nsamples = int(np.sum(seg_durs) / dt)
        solution_interp = np.zeros((qsize, nsamples))

        for i in range(len(interpolators)):
            for j in range(nsamples):
                solution_interp[i, j] = interpolators[i](dt * j)

        return solution_interp, [dt * j for j in range(nsamples)]




