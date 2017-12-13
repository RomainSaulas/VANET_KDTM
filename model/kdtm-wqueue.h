/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef KDTM_WQUEUE_H
#define KDTM_WQUEUE_H

#include <vector>
#include <map>
#include <list>
#include <iostream>

#include "ns3/ipv4-routing-protocol.h"
#include "ns3/simulator.h"
#include "kdtm-packet.h"
#include "ns3/enum.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"


namespace ns3 {
namespace kdtm {

class QueueEntry
{
public:
	QueueEntry (Vector position = Vector (0.0,0.0,0.0), 
		Time backofftime = Seconds (0.0), 
		Ptr<Packet> packet = Create<Packet> (0), 
		uint32_t sourceId = 0, 
		uint32_t messageId = 0, 
		uint32_t prevHopId = 0, 
		uint32_t hopCount = 0,
		bool forwarded = false);

	bool operator== (QueueEntry const & entry) const
	{
		if (m_messageId == entry.GetMessageId ()
			&& m_prevHopId == entry.GetPrevHopId ())
			{
				return true;
			}
		return false;
	}

	Vector GetPosition () const
	{
		return m_position;
	}
	void SetPosition (Vector position) 
	{
		m_position = position;
	}

	Time GetBackOffTime () const
	{
		return m_backOffTime - Simulator::Now ();
	}
	void SetBackOffTime (Time backOffTime) 
	{
		m_backOffTime = backOffTime + Simulator::Now ();
	}

	Ptr<Packet> GetPacket () const
	{
		return m_packet;
	}
	void SetPacket (Ptr<Packet> packet) 
	{
		m_packet = packet;
	}

	uint32_t GetSourceId () const
	{
		return m_sourceId;
	}
	void SetSourceId (uint32_t sourceId) 
	{
		m_sourceId = sourceId;
	}

	uint32_t GetMessageId () const
	{
		return m_messageId;
	}
	void SetMessageId (uint32_t messageId) 
	{
		m_messageId = messageId;
	}

	uint32_t GetPrevHopId () const
	{
		return m_prevHopId;
	}
	void SetPrevHopId (uint32_t prevHopId) 
	{
		m_prevHopId = prevHopId;
	}

	uint32_t GetHopCount () const
	{
		return m_hopCount;
	}
	void SetHopCount (uint32_t hopCount) 
	{
		m_hopCount = hopCount;
	}

	bool GetForwarded () const
	{
		return m_forwarded;
	}
	void SetForwarded (bool forwarded) 
	{
		m_forwarded = forwarded;
	}

private:
	Vector m_position;
	Time m_backOffTime;
	Ptr<Packet> m_packet;
	uint32_t m_sourceId;
	uint32_t m_messageId;
	uint32_t m_prevHopId;
	uint32_t m_hopCount;
	bool m_forwarded;

};

class Queue
{
public:
	/// Callback <Packet, nodeId>
	Queue ();

	Queue(uint32_t maxLen, Time queueTimeOut)
		: m_maxLen (maxLen),
			m_queueTimeOut (queueTimeOut)
	{
	}

	/// Add element to Queue
	void Add (QueueEntry entry);

	/// Delete messageID elements
	void Purge (uint32_t messageId);

	/// Find if entry already in queue
	bool Find (uint32_t messageId, uint32_t prevId);

	/// Find if entry exist
	bool Exist (uint32_t messageId);

	/// Calculate Spatial Distribution
	Vector CalculateSpatialDist (uint32_t setId);

	Time GetQueueTimeOut () const
	{
		return m_queueTimeOut;
	}

	void SetQueueTimeOut (Time timeOut)
	{
		m_queueTimeOut = timeOut;
	}

	QueueEntry & GetEntry (uint32_t messageId)
	{
		return m_queue.find(messageId)->second.front ();
	}

	bool IsAlreadyForwarded (uint32_t messageId)
	{
		if (m_queue.find (messageId) != m_queue.end ())
			{
				return m_queue.find (messageId)->second.front ().GetForwarded ();
			}
		return false;
	}

private:
	uint32_t m_maxLen;
	Time m_queueTimeOut;

	/// Queue of entry: < message id, < sender id, < sender position, backofftimer, Packet>>
	std::map<uint32_t, std::list<QueueEntry>> m_queue;
};

}
}

#endif