# مثال مبسط لمنطق ملف الـ launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join
from get_package_share_directory import get_package_share_directory # type: ignore

def generate_launch_description():
    # 1. تشغيل SLAM Toolbox (لبناء الخريطة)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        )
    )

    # 2. تشغيل Nav2 (للحركة والملاحة)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items() # اجعلها false لو روبوت حقيقي
    )

    return LaunchDescription([
        slam_toolbox,
        navigation
    ])