Livox & Realsense camera calibration by Chanmin 2022-10-18 作

0. git clone https://github.com/hku-mars/livox_camera_calib.git  --> hku_git
   git clone https://github.com/Livox-SDK/livox_ca-mera_lidar_calibration.git --> livox_git
   git clone git@github.com:ycm7089/rosbag_reader.git --> 내 깃허브

1. roslaunch realsense2_camera rs_camera.launch
   roslaunch livox_ros_driver livox_lidar_msg.launch

2. rosbag record /livox/lidar /camera/color/image_raw  (Edge가 있어야하고 근거리는 잘안되는거같음 => 먼거리는 calibration 잘되는듯?)

3. roslaunch camera_lidar_calibration pcdTransfer.launch (livox/lidar -> pcd 파일 저장)
    <param name="input_bag_path"        value="$(find camera_lidar_calibration)/../../data/lidar/" />  <!-- rosbag folder -->
    <param name="output_pcd_path"       value="$(find camera_lidar_calibration)/../../data/pcdFiles/" />  <!-- path to save new pcd files -->
    <param name="data_num"              type="int" value="10" />  <!-- the number of the rosbag -->

    -> 이 부분 각자의 경로와 로스백의 데이터 갯수에 맞게 바꿔주기

4. rosrun pcd_maker bag_to_img_node (/camera/color/image_raw -> bmp 파일 저장)
    std::string bagfile_name = "/home/kimm/livox_front_data.bag"; -> rosbag 경로에 맞게 바꿔주기

5. roslaunch livox_camera_calib multi_calib.launch (hku)
   <multi_calib.yaml>
   common:
       image_path:  "/home/ycj/data/calib/image/old"
       pcd_path:  "/home/ycj/data/calib/pcd/old"
       result_path:  "/home/ycj/data/calib/extrinsic.txt"
       data_num: 3

   camera:
       camera_matrix: [1704.1857 , 0.0,      886.5115,
                       0.0,     1719.5228,  467.9244,
                       0.0,     0.0,      1.0     ]
       dist_coeffs: [-0.0963, 0.0708, 0.0011, -0.0017, 0.000000]

   -> image_path, result_path 는 3,4번에서 진행 하였던 파일 위치 작성
   -> result_path 는 우리가 결과값을 어느 곳에 두고 싶은지 파일 위치 작성
   -> data_num은 사진 갯수라서 나는 1개로 설정함
   -> camera_matrix, dist_coeffs 는 camera calibration 진행 후 얻은 결과 입력 ( Intrinsic Parameter)

   <config_outdoor.yaml>
   ExtrinsicMat: !!opencv-matrix
     rows: 4
     cols: 4
     dt: d
     data: [0.0,   -1.0,   0.0,    0.0,
            0.0,     0.0,  -1.0,    0.0,
            1.0,     0.0,   0.0,    0.0,
            0.0,     0.0,   0.0,    1.0]

   -> lidar camera calibration이 잘 진행이 안된다면 x=(1,4) y=(2,4) z=(3,4) 초기 값 변경시키기

tip : 경로들을 미리 맞춰놓으면 헷갈리지 않으면서 할수있다.
   
