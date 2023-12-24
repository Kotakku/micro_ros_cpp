# micro_ros_cpp
micro-ROSをrclcppライクに書くためのライブラリ

# 開発環境
- Ubuntu22.04
- ROS Humble
- Arduino IDE

# 使い方
ReleaceからzipファイルをダウンロードしてArduino IDEの```Sketch -> Include library -> Add .ZIP Library...```からライブラリを追加する

# Examples
| プログラム名 | 説明 |
| - | - |
| micro_ros_cpp_publisher.ino | int32の値をarduinoからpublishする |
| micro_ros_cpp_subscriber.ino | int32の値をarduinoでsubscribeしてLEDの状態を変化させる |
| micro_ros_cpp_joy_publicher.ino | stringや配列を使う例 |