# tf_start

これは簡単なROS tf2のチュートリアルです．

[参考ページ](http://wiki.ros.org/tf2/Tutorials)

**NOTE : tfとtf2はチュートリアルが異なるので注意すべし**

## 1. static_tf2_broadcaster

1. header config(tf2::staticBroadcasterを宣言)
2. translation config
3. rotation config
4. send

するだけ．

でも，実際の開発ではstaticな変換にこれを使わず，static_transform_publishserをlaunchファイルに描いて使うことのほうが多い．

## 2. tf2_broadcaster

turtlesimを利用して，亀のポーズをtfに送信する方法を学ぶ．

{world} -> {turtle1}のフレーム変換をcartecianとquaternionで受け取る

### topic

topic : TransformStamped

どこから見た座標で，何を示しているのかを付加して送信する．

- header
  - {frame_id} --> {child_id}
- transform
  - translation(cartecian)
  - rotation(quaternion)

### launch

launchファイルでtfにポーズを流すプログラムを起動しているが，シミュレータに現れる亀は１匹なので{turtle1}-->{turtle2}の変換はできない．

## Add frame
フレームを追加する．
やってることはbroadcastと全く同じ気がする

## Tips

RPY --> Quaternion
```
#include <tf2/LinearMath/Quaternion.h>

tf2::Quaternion q;
q.setRPY(rool, pitch, yaw);
```

Quaternion --> RPY
```
#include <tf2/LinearMath/Matrix3x3.h>

tf2::Quaternion q;
...

tf2::Matrix3x3 m(q);
m.getRPY(roll, pitch, yaw);
```
