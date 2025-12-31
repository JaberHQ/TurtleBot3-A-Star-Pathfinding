// enum State
// {
//     GO_TO_POSITION,
//     DRIBBLE,
//     GO_TO_BALL,
//     SHOOT
// };

// enum Position
// {
//     STRIKER,
//     MIDFIELDER,
//     DEFENDER,
//     GOALKEEPER
// };


// class FootballFSM
// {
// public:
//     FootballFSM()
//     {
//         state = GO_TO_POSITION;
//         FSM(state);
//     }

//     void FSM(State state)
//     {
//         ShootOrDribble();

//         if (state == GO_TO_POSITION)
//         {
//             GoToPosition();
//         }

//         if (state == SHOOT)
//         {
//             Shoot();
//         }

//         if (state == DRIBBLE)
//         {
//             Dribble();
//         }
//     }

//     void GoToPosition()
//     {
//         while (m_teammateHasBall)
//         {
//             if (position == STRIKER)
//             {
//                 // if striker is not in position
//                 if (!m_inPosition)
//                 {
//                     // Go to striker coordinates
//                 }
//                 else
//                 {
//                     //return
//                 } 
//             }
//         }
//         if (m_isClosestTeammateToBall)
//         {
//             state = GO_TO_BALL;
//         }
//         else
//         {
//             state = GO_TO_POSITION;
//         }

        
//     }

//     void ShootOrDribble()
//     {
//         if(m_hasBall)
//         {
//             if(m_isInShootingDistance)
//             {
//                 state == SHOOT;
//             }
//             else
//             {
//                 state == DRIBBLE;
//             }
//         }
//     }

//     void Shoot()
//     {
//         // Hit the ball
//         // Shooting timer
//         // If robot is closest teammember to the ball
//         if (m_isClosestTeammateToBall)
//             state == GO_TO_BALL;
//         else
//             state == GO_TO_POSITION;
//     }

//     void Dribble()
//     {
//         // hit ball

//         ShootOrDribble();
//     }

// private:
//     State state;
//     Position position;

//     bool m_hasBall;
//     bool m_isInShootingDistance;
//     bool m_teammateHasBall;
//     bool m_inPosition;
//     bool m_isClosestTeammateToBall;


// };