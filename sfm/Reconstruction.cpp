/*******************************************************
 * Copyright (C) 2020, Navigation And Location Group, Peking University
 *
 * This file is part of IncrementalSfm.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: GJH (guojh_rs@pku.edu.cn)
 *******************************************************/

#include "sfm/config.h"
#include "sfm/reconstruction.h"
#include "sfm/utils.h"

int main(int argc, char **argv) {
    using namespace sfm;

    if (argc != 2) {
        cerr << "Usage: Reconstruction <ConfigFilePath>\n";
        return -1;
    }
    cout << "================================\n";
    cout << "Reconstruction\n";
    cout << "================================\n";

    Config::instance()->setParameterFile(argv[1]);
    string outputDir = Config::instance()->get<string>("output_dir");

    Reconstruction reconstruction;
    reconstruction.run();
    reconstruction.writePLYBinary(Utils::unionPath(outputDir,"gjhAll.ply"));

    return 0;
}