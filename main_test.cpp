#include "mst_ws/multiple_search_and_track.h"

int main(){
    MultipleSearchAndTrack mst;
    mst.init();
    mst.run();
    mst.destory();

    do printf("Jobs done! press q to quit\n");
    while(getchar()!='q');
    return -1;


}