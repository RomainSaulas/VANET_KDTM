/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "kdtm-wqueue.h"
#include "ns3/log.h"
#include <algorithm>


NS_LOG_COMPONENT_DEFINE ("KdtmQueue");

namespace ns3 {
namespace kdtm {

/// QueueEntry
QueueEntry::QueueEntry (Vector position, Time backofftime, 
		Ptr<Packet> packet, 
		uint32_t sourceId, 
		uint32_t messageId, 
		uint32_t prevHopId, 
		uint32_t hopCount,
		bool forwarded)
	:	m_position (position),
		m_backOffTime (backofftime),
		m_packet (packet),
		m_sourceId (sourceId),
		m_messageId (messageId),
		m_prevHopId (prevHopId),
		m_hopCount (hopCount),
		m_forwarded (forwarded)
	{
	}


/// Queue
Queue::Queue ()
{
}

void
Queue::Add (QueueEntry entry)
{
	m_queue[entry.GetMessageId ()].push_front (entry);
}

void
Queue::Purge (uint32_t messageId)
{
	m_queue.erase (messageId);
}

bool 
Queue::Find (uint32_t messageId, uint32_t prevId)
{
	std::map<uint32_t, std::list<QueueEntry>>::iterator i = m_queue.find (messageId);
	if (i != m_queue.end ())
		{
			std::list<QueueEntry>::iterator j = i->second.begin ();
			for (; j!=i->second.end (); j++)
				{
					if (j->GetPrevHopId () == prevId)
						{
							return true;
						}
				}
		}
		return false;
}

bool 
Queue::Exist (uint32_t messageId)
{
	if (m_queue.find (messageId) != m_queue.end ())
		return true;

	return false;
}


Vector 
Queue::CalculateSpatialDist (uint32_t setId)
{
  std::list<QueueEntry> set = m_queue[setId];

	std::list<QueueEntry>::const_iterator i = set.begin ();

	Vector spatialDist;

  spatialDist.x = 0;
  spatialDist.y = 0;

  if (!(set.empty ()))
    {
      double sumx = 0;
      double sumy = 0;

      for (; i != set.end (); i++)
        {
          sumx += i->GetPosition ().x;
          sumy += i->GetPosition ().y;      
        }

      spatialDist.x =  ((double) 1/set.size ()) * sumx;
      spatialDist.y =  ((double) 1/set.size ()) * sumy;  
    }

  return spatialDist;
}

}
}