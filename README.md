# realman 机械臂遥操作

> 已在 realman RM65B, RM63III 上测试通过。

## 功能

- [x] 支持键盘控制末端移动
- [x] 支持 SpaceMouse 控制末端移动
- [x] 支持 SpaceMouse 侧键自定义功能按键
- [x] 支持末端移动速度控制
- [x] 支持通过手动或通过网页端开启/关闭服务

## 主界面

![ui](doc/UI.png)

## 环境配置

```sh
# 1、安装基础库
pip install ./robot_toolbox-*.whl

# 2、安装依赖
pip install -r requirements.txt

# 3、启动主程序
python -m realman_teleop
```
