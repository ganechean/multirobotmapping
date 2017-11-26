/**
 * @author nconlon
 */

#ifndef POINT_HPP
#define POINT_HPP

namespace robot {

    class Point
    {
    public:
        Point(double x, double y);
        double getX();
        double getY();
        double getDistance(Point other);
        double getDistance(double x, double y);


    private:
        double x;
        double y;

    };

}


#endif // POINT_HPP

