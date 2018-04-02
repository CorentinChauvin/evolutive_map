#ifndef CONTAINER_H
#define CONTAINER_H

#include <string>
#include <iostream>

class Container
{
	public:

	Container(double x, double y, double z, double yaw, std::string name);
	void display(std::ostream &flux) const;
	std::string getName();
	double getX();
	double getY();
	double getYaw();

	private:

	double m_x;
	double m_y;
	double m_z;
	double m_yaw;
	std::string m_name;

    friend std::ostream& operator<<(std::ostream& flux, Container const& cont);
};

#endif
