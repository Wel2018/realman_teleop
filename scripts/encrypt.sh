#################################
# Encrypt
# weikang
# 2026-01-09
#################################

TARGET=realman_teleop
OUTPUT=dist/dist_${TARGET}_`date "+%Y-%m-%d-%H.%M.%S"`

echo 加密模块 $TARGET 到输出目录 $OUTPUT
echo 注：忽略 ERROR out of license，不影响程序运行

# 加密模块
echo 加密模块...
pyarmor --silent gen -O ${OUTPUT} \
    -r -i projects/${TARGET}

# 拷贝配置文件
echo 拷贝配置文件...
cp projects/${TARGET}/appcfg.yaml \
    ${OUTPUT}/${TARGET}

# 拷贝机械臂依赖库
echo 拷贝机械臂依赖库...
cp -r projects/${TARGET}/bgtask/common \
    ${OUTPUT}/${TARGET}/bgtask

# 拷贝界面描述
echo 拷贝界面描述...
cp -r projects/${TARGET}/ui \
    ${OUTPUT}/${TARGET}

echo 模块 $TARGET 加密完成！
