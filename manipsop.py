import numpy as np
from autolab_core import RigidTransform
import sys
sys.path.append('/home/hyperpanda/frankapy/')
from frankapy import FrankaArm
import pyrealsense2 as rs
from PIL import Image, ImageDraw
import time
import open3d as o3d
from pyquaternion import Quaternion
def check_axis(rotmat):
    x_axis = rotmat[:,0]
    y_axis = rotmat[:,1]
    z_axis = rotmat[:,2]
    if 0.99<abs(np.dot(x_axis,[1,0,0])):
        return 0
    elif 0.99<abs(np.dot(y_axis,[0,1,0])):
        return 1
    elif 0.99<abs(np.dot(z_axis,[0,0,1])):
        return 2
    else:
        print('CANNOT calulate gripper forward direction')
        assert(0)
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z])
  
def get_distance(pose1, pose2):
    return (pose1[0]- pose2[0]) ** 2 + (pose1[1]- pose2[1]) ** 2 + (pose1[2]- pose2[2]) ** 2

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


def pick_motion(target_quat,world_point):
  '''
  pick along the z axis
  '''
  #go to the up direction of contact point
  random_position = RigidTransform(rotation=target_quat, translation=np.array(world_point[:3]-[0.12,0.06,-0.02]),
      from_frame='franka_tool', to_frame='world')

  fa.goto_pose(random_position) 
  T_ee_world = fa.get_pose()
  #move downward
  random_position = RigidTransform(rotation=target_quat, translation=np.array(T_ee_world.translation-[0,0,0.04]),
    from_frame='franka_tool', to_frame='world')

  fa.goto_pose(random_position) 
  T_ee_world = fa.get_pose()
  print('Translation: {} | Rotation: {}'.format(T_ee_world.translation, T_ee_world.quaternion))

  fa.close_gripper()
  T_ee_world = fa.get_pose()
  T_ee_world.translation += [0, 0, 0.2]
  fa.goto_pose(T_ee_world)
  # fa.open_gripper()
  
def pull_motion(target_quat,world_point,NUM_CONTROL=4,MOVE_DELTA=-0.03):
  print('-----------------establish initial contact')
  fa.open_gripper()
  T_ee_world = fa.get_pose()
  gripper_axis = check_axis(quaternion_to_rotation_matrix(target_quat))
#   gripper_axis = 0
  random_position = RigidTransform(rotation=target_quat, translation=T_ee_world.translation,
        from_frame='franka_tool', to_frame='world')
  fa.goto_pose(random_position) 
  if gripper_axis ==  0: #positive x-axis: forward
    random_position = RigidTransform(rotation=target_quat, translation=np.array(world_point[:3]-[0.17,0.05,0]),
        from_frame='franka_tool', to_frame='world')
    fa.goto_pose(random_position) 
    T_ee_world = fa.get_pose()
    random_position = RigidTransform(rotation=target_quat, translation=np.array(T_ee_world.translation+ [0.08,0,0]),
        from_frame='franka_tool', to_frame='world')
    fa.goto_pose(random_position) 
    
  elif gripper_axis == 2: #negative z-axis: downward
    random_position = RigidTransform(rotation=target_quat, translation=np.array(world_point[:3]-[0.11,0.05,-0.05]),
        from_frame='franka_tool', to_frame='world')
    fa.goto_pose(random_position) 
    # exit()
    T_ee_world = fa.get_pose()
    random_position = RigidTransform(rotation=target_quat, translation=np.array(T_ee_world.translation-[0.0,0.01,0.05]),
        from_frame='franka_tool', to_frame='world')
    fa.goto_pose(random_position) 
#   exit()
  T_ee_world = fa.get_pose()
  print('Translation: {} | Rotation: {}'.format(T_ee_world.translation, T_ee_world.quaternion))
  fa.close_gripper()
#   exit()

  print('-----------------start gradually pulling and adjusting pose')
  for i in range(NUM_CONTROL):
    #calculate the next moving position
    init_pose = fa.get_pose()
    init_quat = init_pose.quaternion
    
    if gripper_axis == 2:
        init_dir = np.array([0, 1, 0]) # grasp edge: gripper direction, z-axis
    elif gripper_axis == 0:
        init_dir = np.array([0, 0, 1]) # grasp handle: gtipper forward direction x-axis
    
    rotation_quaternion = Quaternion(init_quat[0], init_quat[1], init_quat[2], init_quat[3])
    init_dir = Quaternion(0, init_dir[0], init_dir[1], init_dir[2])
    result_quaternion = rotation_quaternion * init_dir * rotation_quaternion.conjugate
    move_dir = [result_quaternion.x, result_quaternion.y, result_quaternion.z]
    move_pos = init_pose.translation + [MOVE_DELTA * val for val in move_dir]
    # print(move_pos,init_pose.translation)
    # exit()
    next_pose = RigidTransform(rotation=init_quat, translation=move_pos,
    from_frame='franka_tool', to_frame='world')
    fa.goto_pose(next_pose)
    
    
    #calculate the adjusted quaternion
    current_pose = fa.get_pose()  
    delta_vec  = current_pose.translation - init_pose.translation
    delta_s2 = get_distance(current_pose.translation,init_pose.translation)
    delta_pos = [MOVE_DELTA * val for val in move_dir]
    delta_l = np.dot(delta_vec, delta_pos) / np.linalg.norm(delta_pos)
    delta_x = np.sqrt(delta_s2 - delta_l ** 2)
    delta_theta = np.arcsin(delta_x * delta_l / delta_s2)
    axis = np.cross(delta_vec, delta_pos)
    final_quat = rotate_quaternion(current_pose.quaternion, axis, -delta_theta)
    print(final_quat,current_pose.quaternion)
    # exit()
    next_pose = RigidTransform(rotation=final_quat, translation=current_pose.translation,
    from_frame='franka_tool', to_frame='world')
    fa.goto_pose(next_pose)
    
    

  
def rotmat_to_quaternion(rotmat):
    # Compute the trace of the rotation matrix
    trace = np.trace(rotmat)

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (rotmat[2, 1] - rotmat[1, 2]) * s
        qy = (rotmat[0, 2] - rotmat[2, 0]) * s
        qz = (rotmat[1, 0] - rotmat[0, 1]) * s
    elif rotmat[0, 0] > rotmat[1, 1] and rotmat[0, 0] > rotmat[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotmat[0, 0] - rotmat[1, 1] - rotmat[2, 2])
        qw = (rotmat[2, 1] - rotmat[1, 2]) / s
        qx = 0.25 * s
        qy = (rotmat[0, 1] + rotmat[1, 0]) / s
        qz = (rotmat[0, 2] + rotmat[2, 0]) / s
    elif rotmat[1, 1] > rotmat[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotmat[1, 1] - rotmat[0, 0] - rotmat[2, 2])
        qw = (rotmat[0, 2] - rotmat[2, 0]) / s
        qx = (rotmat[0, 1] + rotmat[1, 0]) / s
        qy = 0.25 * s
        qz = (rotmat[1, 2] + rotmat[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rotmat[2, 2] - rotmat[0, 0] - rotmat[1, 1])
        qw = (rotmat[1, 0] - rotmat[0, 1]) / s
        qx = (rotmat[0, 2] + rotmat[2, 0]) / s
        qy = (rotmat[1, 2] + rotmat[2, 1]) / s
        qz = 0.25 * s

    return np.array([qw, qx, qy, qz])
  
def depth_image_to_point_cloud(depth_image,color_intrinsics):
    rows, cols = depth_image.shape
    points = []
    rgbd = np.zeros((rows, cols, 3))
    # permutation = np.array(([0,-1,0],[-1,0,0],[0,0,-1]))
    for y in range(rows):
        for x in range(cols):
            depth = depth_image[y, x]
            if depth > 0:
                z_c = depth *0.001  # Convert depth from mm to meters
                x_c = (x -  color_intrinsics.ppx) * z_c / color_intrinsics.fx
                y_c = (y - color_intrinsics.ppy) * z_c / color_intrinsics.fy
                points.append(np.array([x_c, y_c, z_c]))
    xs,ys = np.where(depth_image>0)
    rgbd[xs,ys] = points
    return points, rgbd

def quaternion_to_rotation_matrix(quaternion):
    # Normalize the quaternion
    q = np.array(quaternion) / np.linalg.norm(quaternion)

    w, x, y, z = q[0], q[1], q[2], q[3]

    # Compute rotation matrix elements
    R = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
                  [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
                  [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]])

    return R

'''
fill in the translation and rotation from the camera to the robot base frame 
translation: ​
  x: 1.3372547950306293​
  y: -0.2290531237794947​
  z: 0.6044339841733884​
rotation: ​
  x: -0.5230059648912933​
  y: -0.47659059933832626​
  z: 0.41546711966951394​
  w: 0.571588342939199
x-axis: forward
y-axis: left
z-axis: upward
'''

translation = np.array([-0.29366,-0.41191,0.761311])
rotation_quat = np.array([0.55975,-0.67578,0.38495,-0.28600])
# result = 'The contact point at (638, 526),  the gripper up 3D direction is [1, 0, -50] the gripper left 3D direction is [49, -1, 1]' #tissue
result = 'The contact point at (684, 436),  the gripper up 3D direction is [0, -2, -50] the gripper left 3D direction is [-49, 11, -1]'
lang = 'pull'
rotmat = quaternion_to_rotation_matrix(rotation_quat)
fa = FrankaArm()

# Retrieve the camera to robot base transform
extrinsic_matrix = np.eye(4)
extrinsic_matrix[:3,:3] = rotmat
extrinsic_matrix[:3,3] = translation


#Take image and depth data from realsense
pipeline = rs.pipeline()
config = rs.config()
# Start streaming
pipeline.start(config)
# when startig the camera, the light is so dim, and it needs time to adjust the lighting
time.sleep(2)
frames = pipeline.wait_for_frames()

depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()
color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
# intrinsics_matrix = np.array([[color_intrinsics.fx, 0, color_intrinsics.ppx],[0, color_intrinsics.fy, color_intrinsics.ppy],[0, 0, 1]])

# Convert images to numpy arrays
depth_image = np.asanyarray(depth_frame.get_data()) 

color_image = np.uint8(np.asanyarray(color_frame.get_data()))
height, width  = depth_image.shape

# Stop streaming
pipeline.stop()
image = Image.fromarray(color_image)
image.save('./images/capture.png')
draw = ImageDraw.Draw(image)

x, y = result.split('(')[1].split(')')[0].split(', ')
x = int(x)
y = int(y)
point_2d = [x,y]
d_x, d_y, d_z = result.split('[')[1].split(']')[0].split(', ')
gripper_up_direction_world = np.array([int(d_x)*0.02, int(d_y)*0.02, int(d_z)*0.02])
print('------shape of depth and color images: ', depth_image.shape,color_image.shape, depth_image[y,x])
fd_x, fd_y, fd_z = result.split('[')[2].split(']')[0].split(', ')
gripper_left_direction_world = np.array([int(fd_x)*0.02, int(fd_y)*0.02, int(fd_z)*0.02])
T_ee_world = fa.get_pose()
print('Translation: {} | Rotation: {}'.format(T_ee_world.translation, T_ee_world.quaternion))
init_quat = T_ee_world.quaternion
init_rot = quaternion_to_rotation_matrix(init_quat)
init_left = init_rot[:3, 1]
cos_simi = np.dot(init_left,gripper_left_direction_world)
if cos_simi<0:
  print('------init and pred is not in the same hemispherem, flip the left direction------')
  gripper_left_direction_world *=-1

# point_2d = (702, 496)
draw.ellipse((point_2d[0]-5, point_2d[1]-5, point_2d[0]+5, point_2d[1]+5), fill=(255,0,0,0))

gripper_up_direction_world /= np.linalg.norm(gripper_up_direction_world)
gripper_left_direction_world /= np.linalg.norm(gripper_left_direction_world)
gripper_forward_direction_world = np.cross(gripper_left_direction_world, gripper_up_direction_world)
gripper_forward_direction_world /= np.linalg.norm(gripper_forward_direction_world)
gripper_left_direction_world = np.cross(gripper_up_direction_world, gripper_forward_direction_world)
gripper_left_direction_world /= np.linalg.norm(gripper_left_direction_world)
contact_rotmat = np.eye(4).astype(np.float32)
contact_rotmat[:3, 0] = gripper_forward_direction_world
contact_rotmat[:3, 1] = gripper_left_direction_world
contact_rotmat[:3, 2] = gripper_up_direction_world

image.save('./images/capture_draw.png')
# exit()
print(color_intrinsics.fx,color_intrinsics.fy,color_intrinsics.ppx,color_intrinsics.ppy)

pcd = o3d.geometry.PointCloud.create_from_depth_image(
            o3d.geometry.Image(depth_image),
            o3d.camera.PinholeCameraIntrinsic(
                width=depth_image.shape[1],
                height=depth_image.shape[0],
                fx=color_intrinsics.fx,
                fy=color_intrinsics.fy,
                cx=color_intrinsics.ppx,
                cy=color_intrinsics.ppy
            ),
            np.linalg.inv(extrinsic_matrix)
        )

point_world_all = np.ones((height,width,4))
xs,ys=np.where(depth_image>0)
point_world_all[xs,ys,:3] = np.array(pcd.points)


world_point = point_world_all[point_2d[1],point_2d[0],:]
# print(world_point)
# exit()

target_quat = rotmat_to_quaternion(contact_rotmat[:3,:3])


if lang == 'pick':
  pick_motion(target_quat,world_point)
elif lang == 'pull':
  pull_motion(target_quat,world_point,NUM_CONTROL=4,MOVE_DELTA=-0.03)




