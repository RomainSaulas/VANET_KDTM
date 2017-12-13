#ifndef KDTM_PTABLE_H
#define KDTM_PTABLE_H

#include <map>
#include <cassert>
#include <stdint.h>
#include "ns3/ipv4.h"
#include "ns3/timer.h"
#include <sys/types.h>
#include "ns3/node.h"
#include "ns3/node-list.h"
#include "ns3/mobility-model.h"
#include "ns3/vector.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/random-variable-stream.h"
#include <complex>

namespace ns3 {
namespace kdtm {

/*
 * \ingroup kdtm
 * \brief Position table used by kDTM
 */
class PositionTable
{
public:
  /// c-tor
  PositionTable ();
  PositionTable (double maxRange, Vector position, Vector velocity);

  /**
   * \brief Gets the last time the entry was updated
   * \param id uint32_t to get time of update from
   * \return Time of last update to the position
   */
  Time GetEntryUpdateTime (uint32_t id);

  /**
   * \brief Adds entry in position table
   */
  void AddEntry (uint32_t id, Vector position, Vector velocity, Time time, double Betaj, Time tj);

  /**
   * \brief Deletes entry in position table
   */
  void DeleteEntry (uint32_t id);

  /**
   * \brief Gets position from position table
   * \param id uint32_t to get position from
   * \return Position of that id or NULL if not known
   */
  Vector GetPosition (uint32_t id);

  /**
   * \brief Checks if a node is a neighbour
   * \param id uint32_t of the node to check
   * \return True if the node is neighbour, false otherwise
   */
  bool isNeighbour (uint32_t id);

  /**
   * \brief remove entries with expired lifetime
   */
  void Purge ();

  /**
   * \brief clears all entries
   */
  void Clear ();

  /**
   * \Get Callback to ProcessTxError
   */
  Callback<void, WifiMacHeader const &> GetTxErrorCallback () const
  {
    return m_txErrorCallback;
  }

  /**
   * \brief Calculate distance Threshold Mc based on Position predicaition algorithm
   */ 
  double CalculateThreshold (Time time);

  bool IsInSearch (uint32_t id);

  bool HasPosition (uint32_t id);

  static Vector GetInvalidPosition ()
  {
    return Vector (-1, -1, 0);
  }

  double GetMaxRange () const {
    return m_maxRange;
  }

  void SetMaxRange (double maxRange) 
  {
    m_maxRange = maxRange;
  }

  Vector GetMyPosition () const {
    return m_myPosition;
  }

  void SetMyPosition (Vector position) 
  {
    m_myPosition = position;
  }

  Vector GetMyVelocity () const {
    return m_myVelocity;
  }

  void SetMyVelocity (Vector velocity) 
  {
    m_myVelocity = velocity;
  }

  double GetPoissonCoeff () const {
    return m_poissonCoeff.second;
  }

  void SetPoissonCoeff (double time)
  {
    m_poissonCoeff.second = (m_poissonCoeff.first * m_poissonCoeff.second + time) / (m_poissonCoeff.first + 1);
    m_poissonCoeff.first++;
  }

  Time GetTrajectoryBegin () const {
    return m_trajectoryBegin;
  }

  void SetTrajectoryBegin (Time time)
  {
    m_trajectoryBegin = time;
  }

  double GetAlpha () const {
    return m_alpha;
  }

  void SetAlpha (double alpha)
  {
    m_alpha = alpha;
  }

  void Print (std::ostream & os);

  double CalculateDegree (Time time);

private:
  Time m_entryLifeTime;
  //  map: node Id  <Position, velocity, time_from, Time_to, Betaj, tj>
  std::map<uint32_t, std::tuple<Vector, Vector, Time, Time, double, Time>> m_table;
  // TX error callback
  Callback<void, WifiMacHeader const &> m_txErrorCallback;

  Vector m_myPosition;
  Vector m_myVelocity;

  double m_maxRange;

  double m_alpha;

  std::pair<uint32_t, double> m_poissonCoeff;

  Time m_trajectoryBegin;

  // Process layer 2 TX error notification
  void ProcessTxError (WifiMacHeader const&);

  /// Calucale link power equation Pij(t) = Aij*t^2 + Bij*t + Cij
  double CalculateAij (Vector velocity);
  double CalculateBij (Vector position, Vector velocity);
  double CalculateCij (Vector position);
  
  /// Calculate time tij(to) and tij(from)
  std::pair<Time, Time> CalculateTimeFromTo (Time time, Vector position, Vector velocity);

  /// Calculate stability of liaison ij: pij(t)
  double CalculateStability (double time, double tj, double Betaj);
  /// Calculate degree of liason ij: Degij(t)
  double CalculateDoubleSigmoid (double t_from, double t_to, double t);

};

std::ostream & operator<< (std::ostream & os, PositionTable & h);

} // kdtm
} // ns3
#endif /* kDTM_PTABLE_H */
