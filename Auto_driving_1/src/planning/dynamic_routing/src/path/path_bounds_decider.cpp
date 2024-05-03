#include "path_bounds_decider.h"
/*
PathBoundsDecider是负责计算道路上可行区域边界的类，其产生的结果如下面代码所示，
是对纵向s等间隔采样、横向d对应的左右边界。这样，即限定了s的范围，又限定了d的范围。

PathBoundsDecider类的主要工作在Process()中完成，在该函数中，分4种场景对PathBound进行计算，
按处理的顺序分别是fallback、pull over、lane change、regular，regular场景根据是否lane borrow又分为
no borrow、left borrow、right borrow，这3种子场景是在一个函数内处理的。
之所以要不同场景对应不同的处理方式，我认为是在不同的场景下，自车会有不同的决策和行为，因此要考察的纵向和横向的范围就不一样，
在计算时也要考虑不同的环境模型上下文。要注意的是，fallback对应的PathBound一定会生成，其余3个场景只有1个被激活，成功生成PathBound后退出函数。

*/

//-------------------------------------------------我把特殊情况跳过了------------------------------------//

PathBoundsDecider::PathBoundsDecider()

{
}

void PathBoundsDecider::Process(ReferenceLineInfo* const reference_line_info)
{
}