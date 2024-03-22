import sys
sys.path.append('/home/hyperpanda/frankapy/')
from frankapy import FrankaArm
from autolab_core import RigidTransform
import numpy as np
import time
def quaternion_to_rotation_matrix(quaternion):
    # Normalize the quaternion
    q = np.array(quaternion) / np.linalg.norm(quaternion)

    w, x, y, z = q[0], q[1], q[2], q[3]

    # Compute rotation matrix elements
    R = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
                  [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
                  [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]])

    return R
fa = FrankaArm()

# print(T_ee_world.quaternion,quaternion_to_rotation_matrix(T_ee_world.quaternion))
# exit()
fa.open_gripper()
T_ee_world = fa.get_pose()
best_quat = np.array([ 0.20756189,  0.67559761, -0.22032635,  0.67226274])
best_pose = RigidTransform(rotation=best_quat, translation=T_ee_world.translation,
    from_frame='franka_tool', to_frame='world')
fa.goto_pose(best_pose)
print(fa.get_pose().quaternion)
exit()
# # time.sleep(3)
fa.close_gripper()
# exit()
# T_ee_world = fa.get_pose()
# T_ee_world.translation += [0, 0, 0.1]
# fa.goto_pose(T_ee_world)
# joints = fa.get_joints()
# joints[6] += np.deg2rad(10)
# fa.goto_joints(joints)
# # T_ee_world = fa.get_pose()
# # T_ee_world.translation += [-0.1, 0.03, 0]
# # fa.goto_pose(T_ee_world)
# exit()
# # # print(T_ee_world.quaternion)
# # # # exit()
# T_ee_world = fa.get_pose()
# best_quat = np.array([ 0.76815808,  0.05773916,  0.62739648, -0.11387733])
# best_pose = RigidTransform(rotation=best_quat, translation=T_ee_world.translation,
#     from_frame='franka_tool', to_frame='world')
# fa.goto_pose(best_pose,joint_impedances=[1500, 1500, 1500, 1250, 1250, 1000, 1000])
# T_ee_world = fa.get_pose()
# print(T_ee_world.quaternion)
# exit()
# T_ee_world = fa.get_pose()
# T_ee_world.translation += [-0, 0.0, 0.15]
# fa.goto_pose(T_ee_world)
# # # fa.close_gripper()
# exit()
# joints = fa.get_joints()
# joints[6] += np.deg2rad(15)
# fa.goto_joints(joints)
def quaternion_from_direction(vector):
    # Normalize the moving direction vector
    norm_vector = vector / np.linalg.norm(vector)

    # Calculate angle of rotation
    theta = np.linalg.norm(vector)

    # Calculate axis of rotation (unit vector)
    axis = norm_vector

    # Construct quaternion
    qw = np.cos(theta / 2)
    qx, qy, qz = axis * np.sin(theta / 2)

    quaternion = np.array([qw, qx, qy, qz])
    return quaternion


    
# def ada_dir_test(dir_set, delta):
#     best_dir = None
#     best_err = 0
#     for dir in dir_set:
        
#         old_pose = fa.get_pose()
#         target_pos = old_pose.translation + dir * delta
#         target_pose = RigidTransform(rotation=old_pose.quaternion, translation=target_pos,
#     from_frame='franka_tool', to_frame='world')
#         # print(target_pos,old_pose.q)
#         fa.goto_pose(target_pose)
#         new_pose = fa.get_pose()
#         delta_pose_norm = np.linalg.norm(new_pose.translation - old_pose.translation)
#         if delta_pose_norm > best_err:
#             best_err = delta_pose_norm
#             best_dir = dir
#     print('--------best dis: ', best_err)
#     return best_dir
# def get_adaptive_dir(init_dir, epsilon):
    
#     # Epsilon is use to scale the random such that its norm is less than epsilon
    
#     # Generate random perturbation for the vector part
#     delta = np.random.randn(3)  # Creates a random vector of size 3
    
#     # Normalize the perturbation
#     delta_norm = np.linalg.norm(delta)
#     delta_normalized = delta / delta_norm
    
#     # Scale it such that its norm is less than epsilon
#     scale = np.random.uniform(0, epsilon)
#     delta_scaled = delta_normalized * scale
    
#     # Add the perturbation to the original quaternion
#     perturbed_dir = init_dir.copy()
#     perturbed_dir[0] += delta_scaled[0]
#     perturbed_dir[1] += delta_scaled[1]
#     perturbed_dir[2] += delta_scaled[2]
#     dir_norm = np.linalg.norm(perturbed_dir)
#     perturbed_dir = perturbed_dir / dir_norm
    
#     return perturbed_dir

# def get_adaptive_dir_set(num, init_dir, epsilon):
#     dir_set = []
#     for i in range(num):
#         dir_set.append(get_adaptive_dir(init_dir, epsilon))
#     return dir_set

# def adjust_rotmat(init_dir):
#     DIR_EPSILON = 0.1
#     NUM_ADA_DIR = 5
#     TEST_MOVE_DELTA = -0.005
    
    
#     dir_set = get_adaptive_dir_set(NUM_ADA_DIR, init_dir, DIR_EPSILON)
#     dir_set.append(init_dir)
#     # print(dir_set)
#     # exit()
#     best_dir = ada_dir_test(dir_set, TEST_MOVE_DELTA)
    
#     return best_dir
# MOVE_DELTA = -0.02
# for i in range(2):
#     T_ee_world = fa.get_pose()
#     cur_pos = T_ee_world.translation
#     print(cur_pos)
#     init_dir = np.array([1, 0, 0])
#     rotation_quaternion = T_ee_world.quaternion
#     rotated_dir = np.quaternion(*rotation_quaternion) * np.quaternion(0, *init_dir) * np.quaternion(*rotation_quaternion.conjugate())
#     init_dir = rotated_dir.imag
#     best_dir = adjust_rotmat(init_dir)
#     T_ee_world = fa.get_pose()
#     ada_pos = T_ee_world.translation
#     next_pos = T_ee_world.translation + best_dir * MOVE_DELTA
#     print(next_pos)
#     next_quat = quaternion_from_direction(best_dir)
#     print(next_quat,T_ee_world.quaternion)
#     target_pose = RigidTransform(rotation=next_quat, translation=next_pos,
#         from_frame='franka_tool', to_frame='world')
#             # print(target_pos,old_pose.q)
#     fa.goto_pose(target_pose)
#     print(fa.get_pose().translation)
from pyquaternion import Quaternion
# def ada_dir_test(dir_set, move_delta, test_delta, quaternion=None):
    # best_dir = None
    # best_err = 0
    
    # for dir in dir_set:
        
    # old_pose = fa.get_pose()
    #     target_pos = old_pose.translation + [test_delta * val for val in dir]

    #     target_pose = RigidTransform(rotation=old_pose.quaternion, translation=target_pos,
    # from_frame='franka_tool', to_frame='world')
    #     fa.goto_pose(target_pose)
    #     new_pose = fa.get_pose()
         
    #     delta_pose_norm = np.linalg.norm(np.concatenate([new_pose.translation,new_pose.quaternion])-np.concatenate([old_pose.translation,old_pose.quaternion]))
    #     print('dir',delta_pose_norm,dir)
    #     if delta_pose_norm > best_err:
    #         best_err = delta_pose_norm
    #         best_dir = dir
    # if best_err == 0:
    #     best_dir = dir_set[-1]
    # best_dir = dir_set[-1]
    # print('select dir', best_dir)
    
    # best_pos = old_pose.translation + [move_delta * val for val in best_dir]
    
    # return best_pos, quaternion, best_dir
def get_distance(pose1, pose2):
    return (pose1[0]- pose2[0]) ** 2 + (pose1[1]- pose2[1]) ** 2 + (pose1[2]- pose2[2]) ** 2
def rotate_quat(quat, axis, angle):
    quat = np.array(quat)
    axis = np.array(axis)
    axis = axis / np.linalg.norm(axis)
    half_angle = angle/2
    rotation_quat = np.array([np.cos(half_angle), axis[0]*np.sin(half_angle), axis[1]*np.sin(half_angle), axis[2]*np.sin(half_angle)])
    w1,x1,y1,z1 = rotation_quat
    w2,x2,y2,z2 = quat
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w,x,y,z])
def quaternion_from_points(pointA, pointB):
    # Calculate translation vector
    v = np.array(pointB) - np.array(pointA)

    # Calculate angle of rotation
    theta = np.arccos(np.dot(pointA, pointB) / (np.linalg.norm(pointA) * np.linalg.norm(pointB)))

    # Calculate axis of rotation
    u = np.cross(pointA, pointB)
    u /= np.linalg.norm(u)

    # Construct quaternion
    qw = np.cos(theta / 2)
    qx, qy, qz = u * np.sin(theta / 2)

    quaternion = np.array([qw, qx, qy, qz])
    return quaternion
def get_ada_dir(move_dir, dir_epsilon):
    delta = np.random.randn(3)
    delta_norm = np.linalg.norm(delta)
    delta_normlized = delta / delta_norm
    scale = np.random.uniform(0, dir_epsilon)
    delta_scale = delta_normlized * scale
    pertubed_dir = move_dir
    pertubed_dir[0] += delta_scale[0]
    pertubed_dir[1] += delta_scale[1]
    pertubed_dir[2] += delta_scale[2]
    dir_norm = np.linalg.norm(pertubed_dir)
    pertubed_dir = pertubed_dir / dir_norm
    return pertubed_dir
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z])
def rotate_quaternion(quaternion, axis, angle):
    # 将四元数和轴转换为NumPy数组
    quaternion = np.array(quaternion)
    axis = np.array(axis)

    # 将轴向量归一化为单位向量
    axis = axis / np.linalg.norm(axis)

    # 计算旋转的一半角度
    half_angle = angle / 2

    # 创建表示旋转的四元数
    rotation_quaternion = np.array([np.cos(half_angle),
                                    axis[0] * np.sin(half_angle),
                                    axis[1] * np.sin(half_angle),
                                    axis[2] * np.sin(half_angle)])

    # 进行四元数乘法，得到旋转后的四元数
    # print(rotation_quaternion.shape,quaternion.shape)
    rotated_quaternion = quaternion_multiply(rotation_quaternion, quaternion)

    return rotated_quaternion
# def get_ada_dir_set(num, current_pose, dir_epsilon):
#     dir_set = []
#     rotation_quaternion = current_pose.quaternion
#     init_dir = np.array([0, 0, 1])
#     rotation_quaternion = Quaternion(rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2], rotation_quaternion[3])
#     init_dir = Quaternion(0, init_dir[0], init_dir[1], init_dir[2])
#     result_quaternion = rotation_quaternion * init_dir * rotation_quaternion.conjugate
#     # w,x,y,z = rotation_quaternion
#     # roll = math.atan2(2.0 * (w*x+y*z),1-2*(x**2+y**2))
#     # pitch = math.asin(2*(w*y-z*x))
#     # yaw = math.atan2(2*(w*z+x*y),1-2*(y**2+z**2))
#     move_dir = [result_quaternion.x, result_quaternion.y, result_quaternion.z]
#     # move_dir = [math.cos(roll),math.cos(pitch),math.cos(yaw)]
#     for i in range(num):
#         dir_set.append(get_ada_dir(move_dir,dir_epsilon))
#     dir_set.append(move_dir)
#     return dir_set
def quaternion_cosine_similarity(q1, q2):
    # Compute dot product of quaternions
    dot_product = np.dot(q1, q2)

    # Compute magnitudes (norms) of quaternions
    norm_q1 = np.linalg.norm(q1)
    norm_q2 = np.linalg.norm(q2)

    # Compute cosine similarity
    cosine_similarity = dot_product / (norm_q1 * norm_q2)
    return cosine_similarity


num_control = 1
T_ee_world = fa.get_pose()
    # quaternion = None
best_quat = T_ee_world.quaternion
for i in range(num_control):
        
    if i == 0:
        move_delta = -0.03
    else:
        move_delta = -0.04
    current_pose = fa.get_pose()
    # time.sleep(2)
    init_pose = current_pose

    rotation_quaternion = current_pose.quaternion
    init_dir = np.array([0, 0, 1]) # grasp handle: gtipper forward direction
    # init_dir = np.array([0, 1, 0]) # grasp edge: gripper direction
    rotation_quaternion = Quaternion(rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2], rotation_quaternion[3])
    init_dir = Quaternion(0, init_dir[0], init_dir[1], init_dir[2])
    result_quaternion = rotation_quaternion * init_dir * rotation_quaternion.conjugate
    
    best_dir = [result_quaternion.x, result_quaternion.y, result_quaternion.z]
    print(best_dir)
    best_pos = current_pose.translation + [move_delta * val for val in best_dir]
    print(current_pose.translation,best_pos)
    # exit()
    best_pose = RigidTransform(rotation=best_quat, translation=best_pos,
    from_frame='franka_tool', to_frame='world')
    fa.goto_pose(best_pose)
    current_pose = fa.get_pose()
    
    position2 = np.array(current_pose.translation)
    
    delta_vec  = current_pose.translation - init_pose.translation
    delta_s2 = get_distance(current_pose.translation,init_pose.translation)
    
    delta_pos = [move_delta * val for val in best_dir]
    delta_l = np.dot(delta_vec, delta_pos) / np.linalg.norm(delta_pos)
    
    delta_x = np.sqrt(delta_s2 - delta_l ** 2)
    
    
    delta_theta = np.arcsin(delta_x * delta_l / delta_s2)
    axis = np.cross(delta_vec, delta_pos)
    final_quat = rotate_quaternion(current_pose.quaternion, axis, -delta_theta)
    print(final_quat,current_pose.quaternion)
    print(quaternion_cosine_similarity(final_quat,current_pose.quaternion))
    # exit()
    print('calculate', final_quat,current_pose.quaternion,current_pose.translation,delta_s2,np.dot(delta_vec, delta_pos))
    best_pose = RigidTransform(rotation=final_quat, translation=current_pose.translation,
    from_frame='franka_tool', to_frame='world')
    fa.goto_pose(best_pose)
    print(fa.get_pose().quaternion)
    
    # exit()
    
    
    # best_pose = RigidTransform(rotation=best_quat, translation=current_pose.translation,
    # from_frame='franka_tool', to_frame='world')
    # fa.goto_pose(best_pose)
    
    # if np.isnan(best_quat[0]) or np.isclose(best_quat[0],np.nan, 1e-6):
    #     best_quat = ori_quat
    # print('adjust quat', best_quat,current_pose.quaternion)
    # exit()