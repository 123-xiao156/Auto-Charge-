/*
 * @copyright: (C) 2025, 江苏集萃智能制造技术研究所有限公司
 * @File: main.cpp
 * @Description: 充电节点，实现无人车的寻桩充电和离桩功能
 * @Version: 0.11.1
 * @Author: liuzhiwen
 * @Date: 2025-11-11
 */

#include "power_manage.h"

int main(int argc, char** argv)
{ 
    QApplication a(argc,argv);
    ros::init(argc, argv, "power_manage_node");
    ros::NodeHandle nh;
    PowerManage pm;
    return a.exec();
}
