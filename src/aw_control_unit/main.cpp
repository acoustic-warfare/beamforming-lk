/** @file main.cpp
 * @author Janne, Tuva
 * @date 2024-07-04
*/

#include "aw_control_unit.h"

int main() {
    AWControlUnit controlUnit(true);
    controlUnit.Start({21875}, "10.0.0.1", false, "false",
                      0, true, true, 100, false, false, 180, false, false, false, false);
}