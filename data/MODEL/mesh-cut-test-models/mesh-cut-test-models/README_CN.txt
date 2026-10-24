Mesh Cut 测试模型

01_cube：边长 10，12 个三角形。先测试平切、斜切、连续切割。
02_tetrahedron：4 个三角形。测试斜面和经过尖角的切割。
03_octahedron：8 个三角形。测试经过中心、顶点和边的切割。
04_L_prism：20 个三角形。正面为凹 L 形，沿厚度方向切开可测试凹轮廓封口。
05_cube_multimaterial：与立方体同形，两组材质。用于复现当前逐 subset 切割的限制；切割可能被拒绝，并不代表单材质切割失效。

所有模型以原点为中心附近，范围约 -6..6；均为闭合表面，向外绕序，正索引，三角形面，包含 vt / vn。MTL 使用纯色，不需要贴图。
OBJ 与同名 MTL 必须放在同一目录。建议复制全部文件至 data/MODEL/。

例如 player.cpp：
#define MODEL_PLAYER "data/MODEL/01_cube.obj"

按顺序测试前四个，第五个用于限制检查。已验证边的反向配对及正体积，尚未在你的 DirectX 工程中加载验证。
