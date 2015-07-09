#include "shape2D.h"
#include "IDSReal2D.h"
#include "IDSBox.h"

using namespace Robotics;
using namespace Robotics::GameTheory;

Shape2D::~Shape2D()
{}

Shape2D::Shape2D() 
	: m_box()
{}