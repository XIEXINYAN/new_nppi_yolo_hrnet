# 视觉感知程序说明

## 概述
本分支为version3.0分支，一般情况下禁止直接在分支下提交代码。目前提交的代码需要包含能在本地运行的CMakeLists.txt文件

## 文件说明

该分支下的文件树需要保持如下，不得随意改动文件目录结构：

```
.
├── cmake                      #cmake文件，未评审不得修改
├── main.cpp                   #主函数，未评审不得修改
├── CMakeLists.txt     #cmake文件，未评审不得修改
├── config    #模块配置文件
├── driver    #调试时使用，运行时使用dds driver
├── lib
└── src
    ├── camera_process_thread.cpp            #处理主线程，未评审不得修改
    ├── camera_process_thread.h
    ├── camera_alg
        ├── VideoAlg.cpp                #视频分析类，未评审不得修改
        ├── VideoAlg.h
    │   ├── bifocalfusion     #双摄融合
    │   ├── deepsort          #deepsort跟踪
    │   │   ├── include
    │   │   └── src
    │   ├── detector          #2D检测
    │   ├── detector_3d       #3D检测
    │   ├── distance          #测距
    │   ├── soloseg           #分割
    │   └── tracker           #跟踪
    └── driver                #驱动 ，测试用
```

为减少服务器空间占用，请每次合并代码前删除bin，build，lib，.vscode文件夹下的所有文件，只保留代码和配置文件
发布版本中需要将VideoAlg.h 中的DEBUG宏定义注释掉，系统不会打印大量信息并存储运行时间统计。
## 提交流程

1.测试子分支准备合并的代码，确保能在本地正常运行

2.拉取dev分支，保持分支为最新

3.合并分支，解决冲突

4.更新readme.md（注，子分支更改记录按需求更新，不强制）

## commit编写规范

1.格式

```
<type>: <subject>
例：
fix:用户查询缺少username属性 
```

2.type

```
- feat: 新功能、新特性
- temp:暂存，可用于每日备份等情况，可以存在bug或未开发完的功能
- fix: 修改 bug
- perf: 更改代码，以提高性能（在不影响代码内部行为的前提下，对程序性能进行优化）
- refactor: 代码重构（重构，在不影响代码内部行为、功能下的代码修改）
- docs: 文档修改
- style: 代码格式修改, 注意不是 css 修改（例如分号修改）
- test: 测试用例新增、修改
- build: 影响项目构建或依赖项修改
- revert: 恢复上一次提交
- ci: 持续集成相关文件修改
- chore: 其他修改（不在上述类型中的修改）
- release: 发布新版本
- workflow: 工作流相关文件修改
```

3.subject

subject是commit目的的简短描述，不超过50个字符；使用中文；结尾不加句号或其他标点符号。











