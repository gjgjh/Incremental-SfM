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

#ifndef CONFIG_H
#define CONFIG_H

#include "sfm/common.h"

namespace sfm{

// a Singleton class
class Config {
public:
    ~Config();
    Config(const Config &) = delete;
    Config &operator=(const Config &) = delete;

    static std::shared_ptr<Config> instance();
    void setParameterFile(const std::string &filename);

    template<typename T>
    T get(const std::string &key) const {
        return T(file_->operator[](key));
    }

private:
    Config() {}

    cv::FileStorage *file_;
};

}

#endif // CONFIG_H
