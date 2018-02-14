#ifndef Container_h
#define Container_h

#include <string>
#include <iostream>

class Container
{
	public:
	
	Container(double x, double y, double z, double yaw, std::string nom);
	void afficher(std::ostream &flux) const;
	std::string getName();
	double getX();
	double getY();
	double getYaw();

	private:
	
	double m_x;
	double m_y;
	double m_z;
	double m_yaw;
	std::string m_nom;
	
    friend std::ostream& operator<<(std::ostream& flux, Container const& cont);
};

#endif
