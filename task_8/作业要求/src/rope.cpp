#include <iostream>
#include <vector>
#include <math.h>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"


namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        Vector2D dist = end - start;
        for (int i = 0; i < num_nodes; i++){
            Mass *m;
            m = new Mass(start + dist * i / (num_nodes - 1), node_mass, false); 
            m->velocity = Vector2D(0, 0);
            m->forces = Vector2D(0, 0);         
            masses.emplace_back(m);

            Spring *sp;
            sp = new Spring(m, m, k);
            if(i > 0){
                springs[i-1]->m2 = m;
                springs[i-1]->rest_length = (springs[i-1]->m1->position - m->position).norm();
            }
                 
            if(i < num_nodes-1)
                springs.emplace_back(sp);
        }
        for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
        }

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            double len = (s->m2->position - s->m1->position).norm();
            s->m1->forces += s->k * ( (s->m2->position - s->m1->position) / len ) * (len - s->rest_length);
            s->m2->forces += s->k * ( (s->m1->position - s->m2->position) / len ) * (len - s->rest_length);

        }

        float k_d = 0.005; // this variable affects the display of movement heavily!!!
        for (auto &m : masses)
        {
            if (!m->pinned)
            {                
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                // TODO (Part 2): Add global damping

                m->forces += - k_d * m->velocity; // simplified version   simpler than the one on class
                m->forces += gravity;               

                Vector2D a = m->forces / m->mass;
                m->last_position = m->position;
                //Semi-Implicit Euler Method              s(steps per frame) could be default(64) to get converged!!!                      ./ropesim
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
                //Explicit Euler Method                   s(steps per frame) should be set to the extent like 100,000 to get converged!!!  ./ropesim -s 100000
                // m->position += m->velocity * delta_t;
                // m->velocity += a * delta_t;
            }
        
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            double len = (s->m2->position - s->m1->position).norm();
            if(s->m1->pinned && !s->m2->pinned)
                s->m2->position += (len - s->rest_length) * (s->m1->position - s->m2->position).unit();
            else if(s->m2->pinned && !s->m1->pinned)
                s->m1->position += (len - s->rest_length) * (s->m2->position - s->m1->position).unit();
            else if(!s->m2->pinned && !s->m1->pinned){
                s->m1->position += 0.5 * (len - s->rest_length) * (s->m2->position - s->m1->position).unit();
                s->m2->position += 0.5 * (len - s->rest_length) * (s->m1->position - s->m2->position).unit();
            }
        }

        float damping_factor = 0.00005;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D a = gravity / m->mass;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                m->last_position = m->position;
                m->position += (1-damping_factor) * (m->position - m->last_position) + a * delta_t * delta_t;
            }
        }
    }
}
