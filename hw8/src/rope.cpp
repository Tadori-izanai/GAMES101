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

        masses.resize(num_nodes);
        springs.resize(num_nodes - 1);
        Vector2D segment = (num_nodes > 1) ? (end - start) / (num_nodes - 1.0) : Vector2D();

        for (int i = 0; i < num_nodes; i += 1) {
            masses[i] = new Mass(start + segment * i, node_mass, false);
        }
        for (int i = 0; i < num_nodes - 1; i += 1) {
            springs[i] = new Spring(masses[i], masses[i + 1], k);
        }

//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D aPosition = s->m1->position;
            Vector2D bPosition = s->m2->position;
            double length = (bPosition - aPosition).norm();

            Vector2D forceToA = s->k * (length - s->rest_length) * (bPosition - aPosition) / length;
            s->m1->forces += forceToA;
            s->m2->forces += -forceToA;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;

                // TODO (Part 2): Add global damping
                Vector2D currentVelocity = m->velocity;
                m->forces += -0.02 * currentVelocity;

                Vector2D currentAccelerate = m->forces / m->mass;

                m->velocity += currentAccelerate * delta_t;
//                m->position += currentVelocity * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
//        return;
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D aPosition = s->m1->position;
            Vector2D bPosition = s->m2->position;
            double length = (bPosition - aPosition).norm();

            Vector2D forceToA = s->k * (length - s->rest_length) * (bPosition - aPosition) / length;
            s->m1->forces += forceToA;
            s->m2->forces += -forceToA;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                Vector2D currentAccelerate = m->forces / m->mass;

                // TODO (Part 4): Add global Verlet damping
                double damping_factor = 0.000002;
                m->position += (1 - damping_factor) * (temp_position - m->last_position) + currentAccelerate * delta_t * delta_t;
                m->last_position = temp_position;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
