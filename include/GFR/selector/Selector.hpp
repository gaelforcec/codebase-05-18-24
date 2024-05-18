#pragma once

namespace gfr::selector {

void init(int default_auton, const char** autons);
int get_auton();

} // namespace gfr::selector