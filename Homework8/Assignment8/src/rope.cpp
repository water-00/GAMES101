#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        // 创建结点
        for (int i = 0; i < num_nodes; i++) {
            Vector2D pos = start * ((double)(num_nodes - 1 - i) / (num_nodes - 1)) + end * ((double)i / (num_nodes - 1));
            masses.push_back(new Mass(pos, node_mass, false));
        }
        
        // 创建弹簧
        for (int i = 0; i < num_nodes - 1; i++) {
            springs.push_back(new Spring(masses[i], masses[i + 1], k));
        }

       // Comment-in this part when you implement the constructor
       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {

        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D vec_ab = s->m2->position - s->m1->position;
            Vector2D f = -s->k * vec_ab / vec_ab.norm() * (vec_ab.norm() - s->rest_length); // 若弹簧长度大于原长，f的方向是b受到的力的方向
            s->m1->forces += -f;
            s->m2->forces += f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity;
                Vector2D acc = m->forces / m->mass;
                m->velocity = m->velocity + acc * delta_t;
                m->position = m->position + m->velocity * delta_t;

                // TODO (Part 2): Add global damping 阻尼
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
            Vector2D vec_ab = s->m2->position - s->m1->position;
            Vector2D f = -s->k * vec_ab / vec_ab.norm() * (vec_ab.norm() - s->rest_length);
            s->m1->forces += -f;
            s->m2->forces += f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity;
                Vector2D acc = m->forces / m->mass;
                
                // TODO (Part 4): Add global Verlet damping
                double damping = 0.00005;
                m->position = m->position + (1 - damping) * (m->position - m->last_position) + acc * delta_t * delta_t;
                m->last_position = temp_position;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
