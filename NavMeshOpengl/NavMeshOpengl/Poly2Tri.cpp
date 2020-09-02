#include "Poly2Tri.h"

void BuildSet(string filename)
{
	// string filename = "sample3_in.txt"; //input bdm file name; 	
	Polygon poly(filename, true);
	// //poly.setDebugOption(options.debug);      //set debug flag;
	//三角形化
	poly.triangulation();                   
	// //output results;   
	// //poly.saveAsShowme();
	//输出集合信息
	poly.saveAsMetaPost();
}


// 读入文件 并转换为可走多边形轮廓
bool BuildSet(string filename, rcContourSet &rcConSet)
{
	Polygon poly(filename, true);
	// 三角形化
	poly.triangulation();  
	// 转换
	return poly.ChangeToConset(rcConSet);
}

