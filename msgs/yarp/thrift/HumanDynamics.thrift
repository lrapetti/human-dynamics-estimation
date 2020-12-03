namespace yarp human

/**
 * Representation of a 6D wrench vector
 */
struct Wrench6D {
    1: double x;
    2: double y;
    3: double z;
    4: double tx;
    5: double ty;
    6: double tz;
}

/**
 * Representation of the IHumanDynamics interface
 */
struct HumanDynamics {
    1: list<string> jointNames;
    2: list<double> torques;
    3: list<Wrench6D> internalWrenches;
}
