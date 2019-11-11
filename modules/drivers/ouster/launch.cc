#include "modules/drivers/ouster/include/ouster/os1.h"

int main(int argc, char *argv[]) {
    ouster::OS1::init_client();
    return 0;
}