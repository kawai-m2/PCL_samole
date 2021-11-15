# PCL_samole
pclをROSで実行するためのsample project です


# 環境
ROS-Melodic
realsese-ros
Realsense D435i

# 説明
D435iを使用して，PCLの平面検出をrvizでの視覚化や公式のサンプルを動かすパッケージです．

・path through
 指定した値以外の点群を除去するサンプルです。
 それぞれのx,y,z値の範囲を変更すれば、その値内のものしか表示されないのを確認できます
 
 起動　
 $roslaunch pcl_sample camera.launch
 
 別ターミナルで
 
 $rosrun pcl_sample through
 
 rvizでのtopicから追加
 
 ・平面検出
 ・点群データから、平面となる部分が赤色で出力されるサンプルです。
 
 起動
 $roslaunch pcl_sample camera.launch
 
 別ターミナルで
 
 $rosrun pcl_sample plane
 
  rvizでのtopicから追加
  
  ・path though + 平面感出
  指定した範囲名での平面検出を行います。
  
 $roslaunch pcl_sample camera.launch
 
 別ターミナルで
 
 $rosrun pcl_sample select_plane
  
 
