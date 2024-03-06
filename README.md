# 对地寒假学习任务
## 任务1
### 像素坐标系、相机坐标系、世界坐标系之间的转换关系
世界坐标系即惯性坐标系，可认为静止不动；相机坐标系可经旋转平移转换为世界坐标系；像素坐标系以图片左上角为原点，为二维坐标系。
#### 相机坐标系与至世界坐标系
欧氏变换，由旋转和平移组成：  
以旋转矩阵表示，a<sub>1</sub>=R<sub>12</sub>a<sub>2</sub>+t<sub>12</sub>  
以变换矩阵表示，[a',1]<sup>T</sup>=T[a,1]<sup>T</sup>  
以四元数描述旋转，p'=qpq<sup>-1</sup>  
另有相似变换、仿射变换、射影变换。
#### 像素坐标系
像素坐标系u轴向右水平、v轴向下垂直  
相机的投影计算分为归一化投影、畸变和针孔三步：  
归一：(RP<sub>w</sub>+t)=[X,Y,Z]<sup>T</sup>$\to$[X/Z,Y/Z,1]<sup>T</sup>  
畸变:x'=x(1+k<sub>1</sub>r<sup>2</sup>+k<sub>2</sub>r<sup>4</sup>+k<sub>3</sub>r<sup>6</sup>)+2p<sub>1</sub>xy+p<sub>2</sub>(r<sup>2</sup>+2x<sup>2</sup>)  
    y'=y(1+k<sub>1</sub>r<sup>2</sup>+k<sub>2</sub>r<sup>4</sup>+k<sub>3</sub>r<sup>6</sup>)+p<sub>1</sub>(r<sup>2</sup>+2y<sup>2</sup>)+2p<sub>2</sub>xy  
针孔：Z[u,v,1]<sup>T</sup>=KP  

## 任务2
### 学习相机标定，学习至少一种相机标定方法，并使用matlab或opencv标定一个相机
## 任务3
### 阅读论文《基于无人机巡检的光复区域视觉识别与定位方法研究》（主要4.1和4.2）
## 任务4
### 理清项目中各坐标系的转换关系，制作思维导图
# 学习笔记
## 第三讲
##### 矩阵
a^b为a叉乘b，可看作a^矩阵与b向量的运算，其中a^=[ 0 -a3 a2, a3 0 -a1, -a2 a1 0 ] 
旋转矩阵R：[a<sub>1</sub>, a<sub>2</sub>, a<sub>3</sub>]<sup>T</sup>=Ra', 
R=[e<sub>1</sub><sup>T</sup>,e<sub>2</sub><sup>T</sup>,e<sub>3</sub><sup>T</sup>]<sup>T</sup>*[e'<sub>1</sub>, e'<sub>2</sub>, e'<sub>3</sub>]  
旋转矩阵各分量等于各基向量夹角的余弦值，矩阵充要为行列式为1的正交矩阵，因而其逆（亦即转置）描述相反旋转。  
平移t：t<sub>12</sub>对应坐标系1原点指向坐标系2原点的向量，在坐标1下取的坐标。  
最终得 a'=Ra+t, a<sub>1</sub>=R<sub>12</sub>a<sub>2</sub>+t<sub>12</sub>  
R<sub>12</sub>等下标从右至左读。  
变换矩阵T：[a',1]<sup>T</sup>=T[a,1]<sup>T</sup>，
T为[R,t \n 0<sup>T</sup>,1]。  
旋转向量：方向与旋转轴一致，长度等于旋转角。
罗德里格斯公式：将用旋转向量 $\theta$ n 表示的旋转转换为以旋转矩阵表示：R=cos $\theta$ I+(1-cos $\theta$ )nn<sub>T</sub>+sin $\theta$ n^
欧拉角：以物体绕自身三轴的旋转表示角度，常用的RollPitchYaw角以ZYX顺序旋转。较直观，但存在万向锁问题。
##### 四元数
二维下欲将复平面向量旋转 $\theta$ 角，可乘以e<sup>i $\theta\$ </sup>。e<sup>i $\theta\$ </sup>=cos $\theta\$ + i*sin $\theta\$ 。  
四元数q=q<sub>0</sub>+q<sub>1</sub>i+q<sub>2</sub>j+q<sub>3</sub>k。  
可将ijk视为三轴，ij=k，ji=-k。  
q<sup>-1</sup>=q*/||q<sup>2</sup>||，共轭q*为虚部取反。  
四元数运算性质、旋转表示转换见P58。
## 第五讲
##### 针孔
$\frac{Z}{f}=\frac{X}{X'}=\frac{Y}{Y'}$  
u=f<sub>x</sub>$\frac{X}{Z}$+c<sub>x</sub>  
v=f<sub>y</sub>$\frac{Y}{Z}$+c<sub>y</sub>
Z(u,v,1)<sup>T</sup>=KP,K为相机内参三阶矩阵，P为相机坐标系坐标  
相机位姿又称相机外参，结合位姿得ZP<sub>uv</sub>=K(RP<sub>w</sub>+t)=KTP<sub>w</sub>  
也可将Z维度归一化处理，相当于将式中Z挪至右侧先行与原坐标相除，得到归一化坐标  
##### 畸变
畸变分为径向和切向畸变，分别主要由透镜自身形状和透镜安装误差造成。径向畸变可以多项式表示，得带畸变的归一化坐标  
x'=x(1+k<sub>1</sub>r<sup>2</sup>+k<sub>2</sub>r<sup>4</sup>+k<sub>3</sub>r<sup>6</sup>)  
y'=y(1+k<sub>1</sub>r<sup>2</sup>+k<sub>2</sub>r<sup>4</sup>+k<sub>3</sub>r<sup>6</sup>)  
再加上切向畸变  
x"=x'+2p<sub>1</sub>xy+p<sub>2</sub>(r<sup>2</sup>+2x<sup>2</sup>)  
y"=y'+p<sub>1</sub>(r<sup>2</sup>+2y<sup>2</sup>)+2p<sub>2</sub>xy
