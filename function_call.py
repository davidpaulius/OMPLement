    def ompl_path_planning(
            self,
            target_object: str,
            goal_pose: list[float],
            algorithm: int,
            num_ompl_attempts: int,
            max_compute: int,
            max_simplify: int,
            len_path: int,
            rgb: list[float] = [0.0, 1.0, 0.0],
            draw_path: bool = False,
            ignore_dynamics: bool = False,
        ) -> bool:

        # -- formatting the string name for printing a cool message:
        if target_object:
            self.sim.addLog(self.sim.getInt32Param(self.sim.intparam_verbosity), f'[OMPLement]: finding a plan to object "{target_object}"...')

        # -- create a dummy object that will represent the target goal:
        target_goal = self.sim.createDummy(0.025)
        self.sim.setObjectPose(target_goal, goal_pose, self.sim.getObject(f'/{self.robot_name}'))
        self.sim.setObjectColor(target_goal, 0, self.sim.colorcomponent_ambient_diffuse, rgb)
        self.sim.setObjectColor(target_goal, 0, self.sim.colorcomponent_emission, [0.6, 0.6, 0.6])
        self.sim.setObjectAlias(target_goal, 'OMPL_target')

        # -- we sleep for a bit so we can see this object appear in the sim:
        time.sleep(0.0001)

        ompl_script = self.sim.getScript(self.sim.scripttype_simulation, self.sim.getObject('/OMPLement'))

        path = self.sim.callScriptFunction(
            "ompl_path_planning",
            ompl_script,
            {
                "robot": self.robot_name,
                "goal": target_goal,
                "algorithm": algorithm,
                "num_attempts": num_ompl_attempts,
                "max_compute": max_compute,
                "max_simplify": max_simplify,
                "len_path": len_path,
            },
        )

        if path:
            self.sim.addLog(self.sim.getInt32Param(self.sim.intparam_verbosity), f'[OMPLement]: plan found!')

            # -- we need to disable the IK following done by the "target" dummy of the robot:
            # self.sim.setModelProperty(target, self.sim.modelproperty_scripts_inactive)
            ik_script = self.sim.getScript(self.sim.scripttype_simulation, self.sim.getObject(f"/{self.robot_name}"))
            if ik_script == -1:
                ik_script = self.sim.getScript(self.sim.scripttype_customization, self.sim.getObject(f"/{self.robot_name}"))

            self.sim.setObjectInt32Param(ik_script, self.sim.scriptintparam_enabled, 0)

            self.start()

            # -- use a cubic spline to interpolate time points:
            cs = CubicSpline(
                [0, 0.3, 0.5, 0.8, 1],
                [float('2.5e-3'), float('2.0e-3'), float('1.0e-3'), float('2.0e-3'), float('2.5e-3')]
            )
            xs = np.arange(0, 1, 1/len(path))
            time_points = cs(xs)

            if draw_path: drawn_object = self.sim.callScriptFunction('visualizePath', ompl_script, path, rgb)

            if ignore_dynamics:
                for obj in self.objects_in_sim:
                    obj_handle = self.sim.getObject(f"/{obj}", {"noError": True})
                    if obj_handle != -1:
                        self.sim.setObjectInt32Parameter(obj_handle, self.sim.shapeintparam_static, 1)
                        # self.sim.setObjectInt32Parameter(obj_handle, self.sim.shapeintparam_respondable, 0)

            time.sleep(0.01)

            # -- with the computed path, we will gradually change the configuration of the robot:
            for P in range(len(path)):
                self.sim.callScriptFunction('setConfig', ompl_script, path[P])
                time.sleep(time_points[P])

            time.sleep(0.01)

            if ignore_dynamics:
                for obj in self.objects_in_sim:
                    obj_handle = self.sim.getObject(f"/{obj}", {"noError": True})
                    if obj_handle != -1:
                        self.sim.setObjectInt32Parameter(obj_handle, self.sim.shapeintparam_static, 0)
                        # self.sim.setObjectInt32Parameter(obj_handle, self.sim.shapeintparam_respondable, 1)

            # -- we need to re-enable the IK following done by the "target" dummy of the robot:
            self.sim.setObjectPosition(self.sim.getObject(f'/{self.robot_name}/target'), self.sim.getObjectPosition(self.sim.getObject(f'/{self.robot_name}/tip')), -1)
            self.sim.setObjectOrientation(self.sim.getObject(f'/{self.robot_name}/target'), self.sim.getObjectOrientation(self.sim.getObject(f'/{self.robot_name}/tip')), -1)
            self.sim.setObjectInt32Param(ik_script, self.sim.scriptintparam_enabled, 1)
            if draw_path: self.sim.removeDrawingObject(drawn_object)

        else:
            self.sim.addLog(self.sim.getInt32Param(self.sim.intparam_verbosity), f'[OMPLement]: plan not found!')

        # -- remove the OMPL target object:
        self.sim.removeObjects([self.sim.getObject('/OMPL_target')])

        return bool(path)
