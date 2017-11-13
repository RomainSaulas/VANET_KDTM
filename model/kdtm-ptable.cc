#include "kdtm-ptable.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <algorithm>
#include <cmath>

NS_LOG_COMPONENT_DEFINE ("KdtmTable");


namespace ns3 {
namespace kdtm {

/*
  kdtm position table
*/
PositionTable::PositionTable (double maxRange, Vector position, Vector velocity)
{
  m_txErrorCallback = MakeCallback (&PositionTable::ProcessTxError, this);
  m_entryLifeTime = Seconds (25); //FIXME fix parameter to hello message timer
  
  m_maxRange = maxRange;

  m_myPosition = position;
  m_myVelocity = velocity;

  m_poissonCoeff = std::make_pair(1,15.0);

  m_alpha = 10.0;
}

Time 
PositionTable::GetEntryUpdateTime (uint32_t id)
{
  std::map<uint32_t, std::tuple<Vector, Vector, Time, Time, double, Time> >::iterator i = m_table.find (id);
  return std::get<3> (i->second);
}

/**
 * \brief Adds entry in position table and delete earlier entry if already present
 */
void 
PositionTable::AddEntry (uint32_t id, Vector position, Vector velocity, Time time, double Betaj, Time tj)
{
  std::map<uint32_t, std::tuple<Vector, Vector, Time, Time, double, Time> >::iterator i = m_table.find (id);
  if (i != m_table.end () || id == (i->first))
    {
      m_table.erase (id);
    }

  std::pair<Time, Time> times_from_to = CalculateTimeFromTo (time, position, velocity);

  m_table.insert (std::make_pair (id, 
                        std::make_tuple (position, 
                          velocity, 
                          times_from_to.first, 
                          times_from_to.second,
                          Betaj,
                          tj)));
}

/**
 * \brief Deletes entry in position table
 */
void 
PositionTable::DeleteEntry (uint32_t id)
{
  std::map<uint32_t, std::tuple<Vector, Vector, Time, Time, double, Time> >::iterator i = m_table.find (id);
  if (i != m_table.end ())
    {
        m_table.erase (id);
    }
}

/**
 * \brief Gets position from position table
 * \param id uint32_t to get position from
 * \return Position of that id or NULL if not known
 */
Vector 
PositionTable::GetPosition (uint32_t id)
{

  NodeList::Iterator listEnd = NodeList::End ();
  for (NodeList::Iterator i = NodeList::Begin (); i != listEnd; i++)
    {
      Ptr<Node> node = *i;
      if (node->GetId () == id)
        {
          return node->GetObject<MobilityModel> ()->GetPosition ();
        }
    }
  return PositionTable::GetInvalidPosition ();

}

/**
 * \brief Checks if a node is a neighbour
 * \param id uint32_t of the node to check
 * \return True if the node is neighbour, false otherwise
 */
bool
PositionTable::isNeighbour (uint32_t id)
{

 std::map<uint32_t, std::tuple<Vector, Vector, Time, Time, double, Time> >::iterator i = m_table.find (id);
  if (i != m_table.end () || id == (i->first))
    {
      return true;
    }

  return false;
}


/**
 * \brief remove entries with expired lifetime
 */
void 
PositionTable::Purge ()
{

  if(m_table.empty ())
    {
      return;
    }

  std::list<uint32_t> toErase;

  std::map<uint32_t, std::tuple<Vector, Vector, Time, Time, double, Time> >::iterator i = m_table.begin ();
  std::map<uint32_t, std::tuple<Vector, Vector, Time, Time, double, Time> >::iterator listEnd = m_table.end ();
  
  for (; !(i == listEnd); i++)
    {

      if (GetEntryUpdateTime (i->first) <= Simulator::Now ())
        {
          toErase.insert (toErase.begin (), i->first);

        }
    }
  toErase.unique ();

  std::list<uint32_t>::iterator end = toErase.end ();

  for (std::list<uint32_t>::iterator it = toErase.begin (); it != end; ++it)
    {

      //m_table.erase (*it);
      PositionTable::DeleteEntry (*it);
    }
}

/**
 * \brief clears all entries
 */
void 
PositionTable::Clear ()
{
  m_table.clear ();
}

/**
 * \ProcessTxError
 */
void 
PositionTable::ProcessTxError (WifiMacHeader const & hdr) /// FIXME complete this function
{
}

/**
 * \brief Returns true if is in search for destination
 */
bool 
PositionTable::IsInSearch (uint32_t id)
{
  return false;
}

bool 
PositionTable::HasPosition (uint32_t id)
{
  return true;
}

/**
 * Calculate Threshold based on Distance To Mean method and Kinetic graph method
 */
double 
PositionTable::CalculateThreshold (Time time) 
{
  Purge ();
  if (m_table.empty ())
    {
      return 0;
    }
  double stability;
  double degree;

  double kinetic_degree = 0;

  std::map<uint32_t, std::tuple<Vector, Vector, Time, Time, double, Time>>::const_iterator i = m_table.begin ();
  for (; i != m_table.end (); i++)
    {
      stability = CalculateStability (time.GetDouble (), 
                                      std::get<4> (i->second), 
                                      std::get<5> (i->second).GetDouble ());

      degree = CalculateDoubleSigmoid (std::get<2> (i->second).GetDouble (),
                                       std::get<3> (i->second).GetDouble (),
                                       time.GetDouble ());
      kinetic_degree += stability * degree;
    }

  return 0.86 - 0.95 * exp(-0.13 * kinetic_degree);
}

void 
PositionTable::Print (std::ostream & os) const
{
  std::map<uint32_t, std::tuple<Vector, Vector, Time, Time, double, Time>>::const_iterator i = m_table.begin ();
  while (i != m_table.end ())
    {
      os << "\n id : " << i->first;
      i++;
    }
}

// Private functions
//{


/// Calculate element Aij of equation Pij(t) = Aij*t^2 + Bij*t + Cij
double 
PositionTable::CalculateAij (Vector velocity)
{
  return pow (m_myVelocity.x - velocity.x, 2) + pow (m_myVelocity.y - velocity.y, 2);  
}

/// Calculate element Bij of equation Pij(t) = Aij*t^2 + Bij*t + Cij
double 
PositionTable::CalculateBij (Vector position, Vector velocity)
{
  return 2 * (m_myPosition.x - position.x) * (m_myVelocity.x - velocity.x) 
       + 2 * (m_myPosition.y - position.y) * (m_myVelocity.y - velocity.y);
}

/// Calculate element Cij of equation Pij(t) = Aij*t^2 + Bij*t + Cij
double 
PositionTable::CalculateCij (Vector position)
{
  return pow (m_myPosition.x - position.x, 2) + pow (m_myPosition.y - position.y, 2);
}

/// Calculate neighbors time in and out. Solve the equation Pij(t) = 0
std::pair<Time, Time> 
PositionTable::CalculateTimeFromTo (Time time, Vector position, Vector velocity)
{
  double Aij = CalculateAij (velocity);
  double Bij = CalculateBij (position, velocity);
  double Cij = CalculateCij (position);

  double from;
  double to;

  if (Aij == 0) 
    {
      if (Bij == 0)
        {
          NS_LOG_INFO (" Aij: " << Aij << " Bij: " << Bij << " Cij: " << Cij);
          NS_LOG_INFO (" Time from: " << Seconds (0.0) << " Time to: " << m_entryLifeTime);

          return std::make_pair (time, time + m_entryLifeTime);
        }

      from = - (Cij - pow (m_maxRange, 2)) / Bij;
      to = - (Cij - pow (m_maxRange, 2)) / Bij;

      NS_LOG_INFO (" Aij: " << Aij << " Bij: " << Bij << " Cij: " << Cij);
      NS_LOG_INFO (" Time from: " << from << " Time to: " << to);

      return std::make_pair (time + Seconds (from), time + Seconds (to));
    }

  double delta = pow (Bij, 2) - 4 * Aij * (Cij - pow(m_maxRange, 2));

  if (delta > 0)
    {
      from = (- Bij - sqrt (delta)) / (2 * Aij);
      to = (- Bij + sqrt (delta)) / (2 * Aij);
    }
  if (delta == 0)
    {
      from = - Bij / (2 * Aij);
      to = - Bij / (2 * Aij);

    }
  if (delta < 0)
    {
      from = 0;
      to = 0;
    }

  NS_LOG_INFO (" Aij: " << Aij << " Bij: " << Bij << " Cij: " << Cij);
  NS_LOG_INFO (" Time from: " << from << " Time to: " << to);

  return std::make_pair (time + Seconds (from), time + Seconds (to));
}


/// Calculate double Sigmoid of liason ij with time (t_to and t_from)
double 
PositionTable::CalculateDoubleSigmoid (double t_from, double t_to, double t)
{
  return (1.0 / (1.0 + exp (- m_alpha * (t - t_from)))) * (1.0 / (1.0 + exp (m_alpha * (t - t_to))));
}

/** Calculate stability coefficient of liason ij, pij(t):
 *  pij(t)= exp (-(Betai + Betaj)*(t - (ti*Betai + tj*Betaj)/(Betai + Betaj)))
 */
double
PositionTable::CalculateStability (double time, double tj, double Betaj)
{
  double Betai = 1.0 / m_poissonCoeff.second;
  double ti = m_trajectoryBegin.GetDouble ();

  return exp (-(Betai + Betaj)*(time - ((ti*Betai + tj*Betaj)/(Betai + Betaj))));
}

//}

/// Opreators
//{

std::ostream & operator<< (std::ostream & os, PositionTable const & h)
{
  h.Print(os);
  return os;
}

//}

}   // kdtm
} // ns3
