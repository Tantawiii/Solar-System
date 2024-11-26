#include "box2d/box2d.h"
#include "SFML/Graphics.hpp"
#include <vector>
using namespace sf;
using namespace std;

const float SCALE = 30.0f; // Box2D scale factor: 1 meter = 30 pixels
const float GravForce = 1;
bool flag = false;

int main()
{
    // Create the main window
    sf::RenderWindow window(sf::VideoMode(1000, 800), "SFML + Box2D Example");

    // Initialize Box2D world with gravity
    b2Vec2 gravity(0.0f, 0.0f); // Gravity in the y-axis
    b2World world(gravity);

    // List to store Box2D bodies and their corresponding SFML circles
    vector<b2Body*> bodies;
    vector<CircleShape> circles;
    vector<vector<Vector2f>> trails;

    // Create the dynamic body (circle)
    b2BodyDef bodyDef;
    bodyDef.type = b2_staticBody;
    bodyDef.position.Set(500.0f / SCALE, 400.0f / SCALE); // Starting position in meters
    b2Body* starBody = world.CreateBody(&bodyDef);

    b2CircleShape staticCircle;
    staticCircle.m_radius = 50.0f / SCALE; // Radius in meters

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &staticCircle;
    float starMass = 1.0f;
    fixtureDef.friction = 0.3f; // Surface friction
    starBody->CreateFixture(&fixtureDef);

    CircleShape circle(staticCircle.m_radius * SCALE); // Convert radius to pixels
    circle.setFillColor(Color::Yellow);
    circle.setOrigin(staticCircle.m_radius * SCALE, staticCircle.m_radius * SCALE);

    // Main game loop
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (Mouse::isButtonPressed(Mouse::Left) && flag == false)
            {
                flag = true;
                b2BodyDef planetDef;
                planetDef.type = b2_dynamicBody;
                Vector2i mousePos = Mouse::getPosition(window);
                planetDef.position.Set(mousePos.x / SCALE, mousePos.y / SCALE);
                

                b2Body* body = world.CreateBody(&planetDef);

                b2CircleShape dynamicCircle;
                dynamicCircle.m_radius = 15.0f / SCALE; // Radius in meters

                b2FixtureDef fixtureDef;
                fixtureDef.shape = &dynamicCircle;
                fixtureDef.density = 0.5f; // Mass density
                fixtureDef.friction = 0.3f; // Surface friction
                body->CreateFixture(&fixtureDef);

                CircleShape circle(dynamicCircle.m_radius * SCALE); // Convert radius to pixels
                circle.setFillColor(Color::Blue);
                circle.setOrigin(dynamicCircle.m_radius * SCALE, dynamicCircle.m_radius * SCALE);
                circle.setPosition(mousePos.x, mousePos.y);

                b2Vec2 circlePosition = body->GetPosition();
                b2Vec2 direction = starBody->GetPosition() - circlePosition;

                float length = direction.Length();
                if (length > 0.0f) {
                    direction *= (1.0f / length);
                }

                // Scale the direction vector by the desired force magnitude
                float forceMagnitude = 5.0f; // Adjust as necessary
                b2Vec2 force;
                force.x = - direction.y * forceMagnitude;
                force.y = direction.x * forceMagnitude;

                // Add the body and circle to their respective lists
                body->ApplyForceToCenter(force, true);
                bodies.push_back(body);
                circles.push_back(circle);
                trails.emplace_back();
            }
            // Reset the flag when the mouse button is released
            if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left)
            {
                flag = false;
            }
        }
        // Force = Gravitational Constant * Mass 1  * Mass 2 / Distance^2
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            b2Vec2 position = bodies[i]->GetPosition();
            position -= starBody->GetPosition();
            float forceMag = GravForce * bodies[i]->GetMass() * /*starBody->GetMass()*/ starMass / position.LengthSquared();
            b2Vec2 planetsForce = -position;
            planetsForce *= forceMag / position.Length();
            bodies[i]->ApplyForceToCenter(planetsForce, true);
            trails[i].push_back(sf::Vector2f(bodies[i]->GetPosition().x * SCALE, bodies[i]->GetPosition().y * SCALE));
            if (trails[i].size() > 2000)
                trails[i].erase(trails[i].begin());
        }

        for (size_t i = 0; i < bodies.size(); ++i) {
            if (b2Distance(bodies[i]->GetPosition(), starBody->GetPosition()) < 2.167f) {
                world.DestroyBody(bodies[i]);
                bodies.erase(bodies.begin() + i);
                circles.erase(circles.begin() + i);
                trails.erase(trails.begin() + i);
                --i;
            }
        }

        // Step the Box2D world
        world.Step(1.0f / 60.0f, 6, 2); // 60 Hz, 6 velocity iterations, 2 position iterations

        // Update the SFML circle position based on Box2D body
        b2Vec2 position = starBody->GetPosition();
        circle.setPosition(position.x * SCALE, position.y * SCALE);

        // Update the SFML circles based on Box2D body positions
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            b2Vec2 position = bodies[i]->GetPosition();
            circles[i].setPosition(position.x * SCALE, position.y * SCALE);
        }

        // Clear the window
        window.clear();

        // Draw trails
        for (const auto& trail : trails)
        {
            if (trail.size() > 1) // Only draw if there are enough points
            {
                sf::VertexArray lineStrip(sf::LineStrip, trail.size());
                for (size_t j = 0; j < trail.size(); ++j)
                {
                    lineStrip[j].position = trail[j];
                    lineStrip[j].color = sf::Color::White; // Adjust trail color as needed
                }
                window.draw(lineStrip);
            }
        }

        // Draw the shapes
        window.draw(circle);

        // Draw all the circles
        for (const auto& circle : circles)
        {
            window.draw(circle);
        }

        // Display the rendered frame
        window.display();
    }

    return EXIT_SUCCESS;
}
