//     ____        _     __    test
//    / __ )____  (_)___/ /____
//   / __  / __ \/ / __  / ___/
//  / /_/ / /_/ / / /_/ (__  ) 
// /_____/\____/_/\__,_/____/
//
// I wrote this project in c++ because I wanted to be able to run
// a more graphically detailed simulation.
// It uses the SFML c++ graphics library which can be found here:
// https://www.sfml-dev.org/
// I used the following command to compile the code:
// g++ test.cpp -o boids -lsfml-graphics -lsfml-window -lsfml-system
//
// I realise this was kind of frowned upon but hopefully I don't lose any points.
// I heavily commented the code so it should still be readible for someone
// who is not familiar with c++.

// objects used by the graphics engine
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/ConvexShape.hpp>
#include <SFML/Graphics/PrimitiveType.hpp>
#include <SFML/Graphics/Rect.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/System/Vector3.hpp>

// FILE input/output to log data
#include <bits/types/FILE.h>

// math libraries
#include <math.h>
#include <cmath>

// standard libraries for printing to the console
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>

// vector (similar to numpy array) and list (similar to python lists) 
// data structure libraries
#include <vector>
#include <list>

// string manipulation for command line options
#include <string.h>

class Boid;
#define windowsize (float) 1024
int timescale = 10;                 // how many steps before we update the display
int numboids = 0;                   // counts the number of boids added to the simulation
float aliradius = windowsize*4;     // radius of alignment effect
float alistrength = 0.005;          // strength of the alignment effect
float sepradius = 16;               // radius of separation effect
float sepstrength = 32;             // strength of the separation effect
float cohstrength = .000125;        // strength of the cohesion effect
float boidvelocity = .25;           // how much each boids moves per simulation tick
float connectradius = sepradius*5;  // the maximum radius for which boids are considered "connected" by the graph
std::vector<Boid*> boids; // array of all the boids
sf::RenderWindow* window; // window object

typedef std::vector<Boid*> Cluster; // "Cluster" type used for the network groupings of boids

// macro used to project coordinates of the birds onto the isometric
// plane to be displayed.
#define isoproject(x_, y_, z_) \
    sf::Vector2f((int) (windowsize/2+x_-y_+2*y/3), (int) (windowsize/2+y_+x/3+z_))
    // sf::Vector2f((int) (windowsize/2+x_), (int) windowsize/2+y_) 

// uniform probability distribution between -1 and 1
float uniform() {
    // https://stackoverflow.com/questions/686353/random-float-number-generation
    return 2 * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) - 1;
}
// alternative inputs.  Output is uniform between a and b
float uniform(float a, float b) {
    return (uniform()/2+1)*(b-a) + a;
}
// normal distribution
float normal(float x) {
    return pow(2, -pow(x, 2));
}
// element-wise normal distribution
sf::Vector3f normal(sf::Vector3f v) {
    return sf::Vector3f(normal(v.x), normal(v.y), normal(v.z));
}
// picks the smallest input
float min(float a, float b) {return (a > b) ? b : a;}
// picks the biggest input
float max(float a, float b) {return (a < b) ? b : a;}
// modulus over floating point inputs.  a % b
float mod(float a, float b) {
    if (a >= 0) {return std::fmod(a, b);}
    return std::fmod(a, b) + b;
}
// gets the magnitude of the provided vector
float magnitude(sf::Vector3f v) {
    return sqrt(
        pow(v.x, 2)+
        pow(v.y, 2)+
        pow(v.z, 2)
    );
}
// draws a line between two 3d points, onto the 2d projection in the window
void line(sf::Vector3f v1, sf::Vector3f v2, sf::Color color) {
    float x, y;
    x = y = 0;
    sf::Vertex line[] = {
        sf::Vertex(
            sf::Vector2f((int) (windowsize/2+v1.x-v1.y+2*v1.y/3), (int) (windowsize/2+v1.y+v1.x/3+v1.z-windowsize/6))
        , color),
        sf::Vertex(
            sf::Vector2f((int) (windowsize/2+v2.x-v2.y+2*v2.y/3), (int) (windowsize/2+v2.y+v2.x/3+v2.z-windowsize/6))
        , color)
    };
    window->draw(line, 2, sf::Lines);
}
// draws a line centered at the provided coordinates, with rotation r and distance d
void line(float x, float y, float z, float r, float d, sf::Color color) {
    sf::RectangleShape rect;
    rect.setFillColor(color);
    rect.setSize(sf::Vector2f(1, (int) d));
    rect.setPosition(isoproject(x, y, z));
    rect.setRotation(180*(r)/M_PI);
    window->draw(rect);
}
// other useful forms
void line(float x, float y, float z, float r, sf::Color color) {
    line(x, y, z, r, 50, color);
}
void line(sf::Vector3f v, float r, float d, sf::Color color) {
    line(v.x, v.y, v.z, r, d, color);
}

// the bird class that keeps track of every bird's data
class Boid {
    public:
        int id;                     // the id of this bird
        sf::Vector3f coord;         // the coordinates of this bird
        sf::Vector3f delta;         // the velocity of this bird
        float bsize = 8;            // the size of the bird's triangle on the display
        float ingroupbias;          // how biased this bird is against differently colored birds
        int explorebfs;             // temporary value used in the breadth first search algorithm
        Cluster cluster;            // network cluster that the bird is part of
        // azmth = xy plane rotation
        sf::Color color;            // the bird's color
        sf::ConvexShape shape;      // the bird's triangular shape object for displaying
        sf::ConvexShape shadow;     // the bird's shadow
        std::vector<Boid*> adj;     // a list of birds adjacent to this bird on the network
        std::vector<float> trust;   // this bird's trust of every other bird.
        Boid() {}
        Boid(float x_, float y_, float z_, float bias, sf::Color basecolor) {
            id = numboids++;
            coord.x = x_;
            coord.y = y_;
            coord.z = z_;
            delta.x = uniform()/2; // between -0.5 and 0.5
            delta.y = uniform()/2;
            delta.z = 0; // no starting z velocity
            normalize(); // make the magnitude of the deltas equal to 1
            color = basecolor;  // set the color of the bird
            ingroupbias = bias; // set the bias of the bird

            // setting up the display and graphics for the bird
            shape.setPointCount(4);
            shadow.setPointCount(4);            
            shape.setFillColor(color);
            shadow.setFillColor(sf::Color(64, 64, 64, 32));

            // adding this bird to the global bird list
            boids.push_back(this);
            // set the trust for every bird based on provided ingroupbias
            // python
            // for b in boids:
            for (Boid* & b : boids) {
                if (b != this){ // if the bird in question is not me
                    // set b's trust for me
                    b->trust.push_back(
                        (b->color == color) ?
                            uniform(b->ingroupbias, 1) :   // if we are the same color
                            uniform(0, 1-b->ingroupbias)); // if we are different colors
                    // set my trust for b
                    trust.push_back(
                        (b->color == color) ?
                            uniform(ingroupbias, 1) :   // if we are the same color
                            uniform(0, 1-ingroupbias)); // if we are different colors
                }
                else { // birds trust themselves 100%
                    trust.push_back(1); // set trust in myself to be 1
                }
            }
        }
        // get trust of the provided bird
        float gettrust(Boid* b) {
            return trust.at(b->id);
        }
        // draw a line to another bird
        void line(Boid* b) {
            ::line(coord, b->coord, color-sf::Color(0, 0, 0, 256-16));
        }
        // draw the triangle shape
        void drawdart(sf::ConvexShape* dart, sf::Vector2f base);
        // update the shape and shadow sprite positions and rotations
        void setshape();
        void setshadow();
        // distance to another bird
        float distance(Boid* b) {
            return magnitude(this->coord-b->coord);
        }
        // returns true if connected to b, otherwise false
        bool isconnected(Boid* b) {return distance(b) < connectradius;}
        // gets the list of birds adjacent to this bird
        std::vector<Boid*> getadjacent() {
            adj = std::vector<Boid*>();
            for (Boid* & b : boids) {
                if (isconnected(b)) {
                    adj.push_back(b); // append to adjacency list
                }
            }
            return adj;
        }
        // update bird's coords based on its delta
        void move() {coord += delta*boidvelocity;}
        // set the bird's movement to be magnitude 1
        void normalize() {
            delta /= magnitude(delta);
        }
};

// draw the triangle shape
// graphical stuff not terribly important to the project
void Boid::drawdart(sf::ConvexShape* dart, sf::Vector2f base) {
    float sharpness = 0.8;
    float azmth = atan2(delta.y, delta.x); // angle of rotation
    dart->setPoint(0, base);
    dart->setPoint(1, base+sf::Vector2f(
        cos(azmth+M_PI*sharpness)*bsize,
        sin(azmth+M_PI*sharpness)*bsize
    ));
    dart->setPoint(2, base+sf::Vector2f(
        cos(azmth)*bsize,
        sin(azmth)*bsize
    ));
    dart->setPoint(3, base+sf::Vector2f(
        cos(azmth-M_PI*sharpness)*bsize,
        sin(azmth-M_PI*sharpness)*bsize
    ));
    // ::line(coord.x, coord.y, coord.z, -M_PI, -coord.z, color);
}
// draw the shape of the bird
void Boid::setshape() {
    float x = coord.x;
    float y = coord.y;
    float z = coord.z;
    drawdart(&shape, isoproject(x, y, z-windowsize/6));
}
// draw the shadow of the bird
void Boid::setshadow() {
    float x = coord.x;
    float y = coord.y;
    drawdart(&shadow, isoproject(x, y, windowsize/3));
}

// lists for breadth first search algorithm
std::list<Boid*> explorenext;
std::list<Boid*> explorenow;
// list of all clusters
std::vector<Cluster> clusters;
// get the whole cluster that the provided bird (b) is contained in
Cluster getcluster(Boid* b) {
    Cluster cluster, newcluster;
    cluster.push_back(b);    // cluster.append(b)
    explorenow.push_back(b); // explorenow.append(b)
    b->explorebfs = 1; // set this bird's explored state to be 'explorenow'
    // as long as there are any birds in the 'explorenow' list we will
    // keep going.  Can't use a for loop because birds will be added to the
    // list as we iterate
    while (explorenow.size()) { 
        // get the bird that is next in line
        Boid* p = explorenow.front();
        // loop over all birds adjacent to that first bird
        for (Boid* & a : p->adj) {
            // if the bird is in the unexplored state,
            // set it to the 'explorenext' state and add it to the list
            if (a->explorebfs == 0) {
                a->explorebfs++; // a->explorebfs = explorenext
                explorenext.push_back(a);
            }
        }
        // remove that first bird in line from the line/queue
        // and change them to the 'explored' state.
        explorenow.pop_front();
        p->explorebfs++; // p->explorebfs = explored
        // move all the 'explorenext' birds to the 'explorenow'
        // list and also append them to the current cluster list.
        for (Boid* & a : explorenext) {
            explorenow.push_front(a);
            cluster.push_back(a);
        }
        // reset the explorenext list
        explorenext = std::list<Boid*>();
    }
    // return the cluster we calcualted
    return cluster;
}
// runs getcluster at all birds
float getclusters() {
    Cluster cluster;
    // resets the explorenow and explorenext lists
    explorenow = std::list<Boid*>();
    explorenext = std::list<Boid*>();
    // sets the current cluster tracked to be empty
    clusters = std::vector<Cluster>();
    // set the average cluster size to be 0 to start with
    float avg = 0;
    // set all birds to the 'unexplored' state
    for (Boid* & b : boids) {b->explorebfs = 0;}
    // if a bird is unexplored, get its cluster,
    // otherwise continue to the next bird in the list
    for (Boid* & b : boids) {
        // if b is not unexplored, skip it
        if (b->explorebfs) {continue;}
        cluster = getcluster(b);        // get the cluster for b
        avg += (float) cluster.size();  // track the size of the cluster
        clusters.push_back(cluster); // add the cluster we found to the list of clusters
        for (Boid* & p : cluster) {
            // tell the bird what cluster it is in.
            // p is a bird in the cluster
            p->cluster = cluster;
        }
    }
    // return the average cluster size
    return avg/((float) clusters.size());
}

// draw all the boids and their shadows on the window
void drawboids() {
    for (Boid* & b : boids) {
        b->setshadow();
        window->draw(b->shadow);
    }
    // make sure all the boids are in front of every shadow
    for (Boid* & b : boids) {
        b->setshape();
        window->draw(b->shape);
    }
}

// this keeps the boids in a hemisphere around the center.
// cohesion already accomplishes this, but if cohesion is too low,
// the birds will fly everywhere without being confined.
// applies a force to them when they stray too far.
void confinement() {
    float d;
    for (Boid* & b : boids) {
        // keeps the birds from going below the ground
        if (b->coord.z < 0) {
            b->delta.z += 0.1; // applying a force upwards
        }
        // if the bird is too far, apply a force to push them back
        if (magnitude(b->coord) > windowsize/3) {
            // since the place to push them to is the origin,
            // simply their coordinates are the necessary vector of force to apply
            b->delta -= (b->coord)/((float) 20000);
            // if the bird is too far we set its color to black,
            // to communicate to the viewer
            b->shape.setFillColor(sf::Color::Black);
        }
        else {
            // if the bird was outside and came back we need to
            // set its color back
            b->shape.setFillColor(b->color);
        }
    }
}

// aligning the birds' direction
void alignment() {
    float d, count;
    sf::Vector3f avgdir; // average velocity of the birds
    for (Boid* & b : boids) {
        // reset the counts
        avgdir = sf::Vector3f(0, 0, 0);
        count = 1;
        for (Boid* & p : boids) {
            d = b->distance(p); // calculate the distance to the 2nd bird
            // the strength of alignment is on a normal distribution
            // the bird considers the alignment of all other birds,
            // but it takes closer ones into account more.
            d = 1*normal(d/(aliradius));
            avgdir += d*p->delta;
            count += d;
        }
        avgdir /= count; // getting the average
        // alistrength determines how much the birds are actually
        // willing to adjust their directions.
        // this is a weighted average.
        b->delta = avgdir*alistrength+b->delta*(1-alistrength);
    }
}
// the separation effect
// this is where I really took some liberties.
// the birds are more likely to separate from birds who they distrust
void separation() {
    float d, count;
    for (Boid* & b : boids) {
        for (Boid* & p : boids) {
            d = b->distance(p);
            if (b != p) { // skip the birds if they are the same
                // apply a force based on the trust of the bird we're
                // considering.  Uses a normal distribution as well,
                // similar to alignment.
                b->delta += (
                    sepstrength* // strength multiplier of separation effect for all birds
                    normal(d/(sepradius*(1.5-b->gettrust(p))))*
                    (b->coord-p->coord) // vector between the birds
                );
            }
        }
    }
}
// cohesion effect.
// this uses breadth first search to actually find the center of mass
// of the network cluster.  This probably does not actually have a 
// huge effect on the bird's movement, since the calculated center
// will likely be similar to that calculated by simply doing a radius search,
// but I thought it was an interesting exercise for this project.
void cohesion() {
    float t;
    Cluster cluster;
    sf::Vector3f avgcoord;
    Boid* p;
    int i;
    for (Boid* & b : boids) {
        // reset the vector average
        avgcoord = sf::Vector3f(0, 0, 0);
        // this is a big source of ineffeciency,
        // because now we're calculating clusters for every bird even
        // if we already covered its cluster when we did another bird.
        // Since I'm telling the birds what cluster they are in in getclusters,
        // I should be able to just use cluster = b->cluster, but that resulted
        // in slightly different behavior and I am more confident that this
        // method is mathematically correct instead.
        cluster = getcluster(b);
        // looping over every bird in the cluster and calculating their
        // average coordinate to get the center.
        for (Boid* & p : cluster) {
            avgcoord += p->coord;
        }
        avgcoord /= (float) cluster.size();
        // applying the calculated force with the cohesion strength multiplier
        b->delta -= avgcoord*cohstrength;
    }
}

// move the boids by one simulation tick.
void moveboids() {
    for (Boid* & b : boids) {b->normalize();}
    for (Boid* & b : boids) {b->move();}
}
// generate a random cluster
void createcluster(int count, int x, int y, int z, int radius, float ingroupbias, sf::Color color) {
    for (int i = 0; i < count; i++) {
        new Boid(x+radius*uniform(), y+radius*uniform(), z+radius*uniform(), ingroupbias, color);
    }
}
// generate a random color
sf::Color randomcolor() {
    return sf::Color(uniform(0, 128), uniform(0, 128), uniform(0, 128));
}
// more ways to generate a random cluster.
void randomcluster(float ingroupbias, sf::Color color) {
    createcluster(
        20,
        uniform()*200,
        uniform()*200,
        uniform()*200,
        uniform(25, 50),
        ingroupbias,
        color
    );
}
void randomcluster(float ingroupbias) {
    randomcluster(
        ingroupbias,
        randomcolor()
    );
}

// one step of the simulation
float simulate() {
    for (Boid* & b : boids) {b->getadjacent();}
    float count = getclusters();
    cohesion();
    alignment();
    separation();

    confinement();
    moveboids();
    return count;
}

int main(int argc, char** argv) {

    // seeding the random number generator.
    // removing this line causes the simulation to run exactly
    // the same every time.
    srand(time(NULL));

    // setting the ingroupbias based on the provided command-line argument
    // atof is like calling float("1.23456") in python
    float ingroupbias = atof(argv[1]);
    // creating clusters in different areas of the simulation
    createcluster(25, 200, 200, 200, 25, ingroupbias, randomcolor());
    createcluster(25, 200, 200, -200, 25, ingroupbias, randomcolor());
    createcluster(25, 200, -200, 200, 25, ingroupbias, randomcolor());
    // createcluster(25, 200, -200, -200, 25, ingroupbias, randomcolor());
    // createcluster(25, -200, 200, 200, 25, ingroupbias, randomcolor());
    // createcluster(25, -200, 200, -200, 25, ingroupbias, randomcolor());

    // handling the command line arguments
    // if the argument "graphics" is not provided, the window will not render
    if (argc > 2 && strcmp(argv[2], "graphics") == 0) {
        // handling graphics
        window = new sf::RenderWindow(sf::VideoMode(windowsize, windowsize), "463");
        sf::Event event;
        int i = 0;
        // handling the window so that the viewer can close it without
        // it misbehaving.
        while (window->isOpen()) {
            // handling the window close event
            window->pollEvent(event);
            if (event.type == sf::Event::Closed) {window->close();}
            
            // stepping the simulation
            simulate();
            
            // only update the screen every `timescale` steps of the simulation
            if (i%timescale == 0) {
                window->clear(sf::Color(0xf3f4f7ff));
                drawboids(); // draw all the birds
                for (Boid* & b : boids) {
                    for (Boid* & p : boids) {
                        // draw all the lines between birds
                        if (b->distance(p) < connectradius) {
                            b->line(p);
                        }
                    }
                }
                // update the display
                window->display();
            }
            i++;
        }
    }
    else {
        int previ;
        float cc, prevclustercount;
        prevclustercount = 0;
        previ = 0;
        FILE* f = fopen("data.txt", "a");
        fprintf(f, "[\n    ");
        // run for 40,000 steps (about 1.5 minutes of simulation on my pc)
        for (int i = 0; i < 40000; i++) {
            cc = simulate();
            if (prevclustercount != cc) {
                // store results to a file
                fprintf(f, "[%d, %f], ", i-previ, cc);
                previ = i;
            }
            prevclustercount = cc;
        }
        fprintf(f, "\n],\n");
        fclose(f);
    }
    return 0;
}
