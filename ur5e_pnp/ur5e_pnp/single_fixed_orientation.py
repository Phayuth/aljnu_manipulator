import numpy as np
from spatial_transformation import RigidBodyTransformation as rbt


class SingleFixedOrientationGraspPose:
    """
    [Summary] : Given single grasp point (x,y,z) on object,
    the method produces Grasp and Pregrasp Pose of object in form of Transformation Matrix.
    Poses are represented in Camera's Frame.

    [Method] :

    - Oriention is fixed with z-axis pointing out in one direction.
    - Pregrasp Pose is offseted from Grasp Pose backwards in a fixed distance(m).

    """

    def __init__(self, graspPoint, distanceOffset=0.1) -> None:
        self.graspPoint = graspPoint
        self.preGraspPoint = self.graspPoint - np.array([distanceOffset, 0.0, 0.0])
        self.rotationMatrix = rbt.roty(np.deg2rad(90)) @ rbt.rotz(np.deg2rad(-90))
        self.graspPose = rbt.conv_rotmat_and_t_to_h(self.rotationMatrix, self.graspPoint)
        self.preGraspPose = rbt.conv_rotmat_and_t_to_h(self.rotationMatrix, self.preGraspPoint)

    def get_grasp_and_pregrasp_poses(self):
        return self.graspPose, self.preGraspPose


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from pytransform3d.transformations import plot_transform
    from pytransform3d.plot_utils import make_3d_axis

    xyz = np.array([1, 1, 1])
    gp = SingleFixedOrientationGraspPose(xyz, distanceOffset=0.2)
    grasp, preGrasp = gp.get_grasp_and_pregrasp_poses()

    ax = make_3d_axis(ax_s=1, unit="m", n_ticks=6)
    plot_transform(ax=ax, name="camera_link")
    plot_transform(ax, A2B=grasp, s=0.1, name="grasp")
    plot_transform(ax, A2B=preGrasp, s=0.1, name="pregrasp")
    plt.tight_layout()
    plt.show()