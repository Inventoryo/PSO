#include "mst_ws/multiple_search_and_track.h"

int main(int argc, char **argv){
    string config_path = argv[1];

    MultipleSearchAndTrack mst;
    mst.init(config_path);
    mst.run();
    mst.destory();

    do printf("Jobs done! press q to quit\n");
    while(getchar()!='q');
    return -1;
}