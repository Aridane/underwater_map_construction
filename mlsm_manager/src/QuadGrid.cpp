#include "QuadGrid.h"

//
//
//
//

QuadGrid::QuadGrid(){
}

QuadGrid::~QuadGrid(){

}

QuadGrid::QuadGrid(int pxSize, int pySize, int nxSize, int nySize):
		gridPP_(boost::extents[pxSize][pySize]),
		gridNN_(boost::extents[nxSize][nySize]),
		gridPN_(boost::extents[pxSize][nySize]),
        gridNP_(boost::extents[nxSize][pySize]),
    pxSize_(pxSize), pySize_(pySize), nxSize_(nxSize), nySize_(nySize) {



}


/*void QuadGrid::init(){
	resolution_ = 0.3;
	spanX_ = DEFAULTSIZEXMETERS / resolution_;
	spanY_ = DEFAULTSIZEYMETERS / resolution_;
	grid_ = grid_type(boost::extents[spanX_*2.0][spanY_*2.0]);
}*/
//
// Centre of grid is origin of the world
//
//
//
cell* QuadGrid::operator() (const int x, const int y){
	int p = 0;
	int n = 0;
    //Out of range access will trigger an assertion fail due to
    //range checking inside the multi array
    //++
	if ((x >= 0) && (y >= 0)){
        //gridPP_.shape();
		return &(gridPP_[x][y]);
	}//--
	else if ((x < 0) && (y < 0)){
		return &(gridNN_[-x][-y]);
	}//+-
	else if ((x >= 0) && (y < 0)){
		return &(gridPN_[x][-y]);
	}//-+
	else if ((x < 0) && (y >= 0)){
		return &(gridNP_[-x][y]);
	}
}

int QuadGrid::getXSize(){
    return 0;
}

int QuadGrid::getYSize(){
    return 0;
}
