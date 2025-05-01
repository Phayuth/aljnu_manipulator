import PyKDL as kdl


class KDLChain:

    def __init__(self):
        self.chain = None
        self.tree = None
        self.jointnum = None
        self.root = None
        self.tip = None

        self.fk_solver = None
        self.ik_solver = None

    def config_kdl_kinemtic(self, kdl_tree, root, tip):
        self.tree = kdl_tree
        self.root = root
        self.tip = tip
        self.chain = self.tree.getChain(self.root, self.tip)
        self.jointnum = self.chain.getNrOfJoints()

        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)

    def forward_kinematics(self, joint_positions):
        if len(joint_positions) != self.jointnum:
            raise ValueError("Joint positions length does not match the number of joints in the chain.")

        joint_array = kdl.JntArray(self.jointnum)
        for i in range(self.jointnum):
            joint_array[i] = joint_positions[i]

        end_effector_frame = kdl.Frame()
        self.fk_solver.JntToCart(joint_array, end_effector_frame)

        return end_effector_frame

    def inverse_kinematics(self, target_frame, initial_joint_positions):
        initial_joint_array = kdl.JntArray(self.jointnum)
        for i in range(self.jointnum):
            initial_joint_array[i] = initial_joint_positions[i]

        final_joint_positions = kdl.JntArray(self.jointnum)
        self.ik_solver.CartToJnt(initial_joint_array, target_frame, final_joint_positions)

        return final_joint_positions

    def make_joint_array(self, joint_positions):
        joint_array = kdl.JntArray(self.jointnum)
        for i in range(self.jointnum):
            joint_array[i] = joint_positions[i]

        return joint_array

    def make_frame(self, position, orientation):
        frame = kdl.Frame()
        frame.p = kdl.Vector(
            position[0],
            position[1],
            position[2],
        )
        frame.M = kdl.Rotation.Quaternion(
            orientation[0],
            orientation[1],
            orientation[2],
            orientation[3],
        )

        return frame

    def make_joint_list(self, joint_positions):
        if len(joint_positions) != self.jointnum:
            raise ValueError("Joint positions length does not match the number of joints in the chain.")

        joint_list = []
        for i in range(self.jointnum):
            joint_list.append(joint_positions[i])

        return joint_list

    def make_position_orientation(self, end_effector_frame):
        position = [
            end_effector_frame.p[0],
            end_effector_frame.p[1],
            end_effector_frame.p[2],
        ]
        orientation = [
            end_effector_frame.M.GetQuaternion()[0],
            end_effector_frame.M.GetQuaternion()[1],
            end_effector_frame.M.GetQuaternion()[2],
            end_effector_frame.M.GetQuaternion()[3],
        ]

        return position, orientation
