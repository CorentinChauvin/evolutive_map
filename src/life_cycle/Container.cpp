#include <string>
#include <iostream>
#include "Container.h"

using namespace std;

Container::Container(double x, double y, double z, double yaw, std::string name): m_x(x), m_y(y), m_z(z), m_yaw(yaw), m_name(name)
{
}

string Container::getName()
{
	return m_name;
}

double Container::getX()
{
	return m_x;
}

double Container::getY()
{
	return m_y;
}

double Container::getYaw()
{
	return m_yaw;
}

void Container::display(ostream &flux) const
{
    flux << "  -name:" << m_name << "  -x:" << m_x << "  -y:" << m_y << "  -z:" << m_z << "  -yaw:" << m_yaw;
}

ostream& operator<<(ostream &flux, Container const& cont)
{
    cont.display(flux);
    return flux;
}
