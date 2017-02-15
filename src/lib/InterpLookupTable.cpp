#include "lib/InterpLookupTable.h"

namespace frc973 {
    
InterpLookupTable::InterpLookupTable() {
}

InterpLookupTable::~InterpLookupTable() {
}

InterpLookupTable &InterpLookupTable::AddPoint(double x, double y) {
    return *this;
}

double InterpLookupTable::LookupPoint(double x) {
    return -5;
}

}
