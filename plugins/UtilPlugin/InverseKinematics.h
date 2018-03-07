/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>

#include <cnoid/src/Util/EigenTypes.h>

namespace cnoid {

template <typename t_matrix>
t_matrix PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6);


}
