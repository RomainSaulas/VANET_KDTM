/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */


#ifndef KDTM_PACKET_H
#define KDTM_PACKET_H

#include <iostream>
#include "ns3/header.h"
#include "ns3/enum.h"
//#include "ns3/ipv4-address.h"
#include <map>
#include "ns3/nstime.h"

namespace ns3 
{
namespace kdtm 
{


enum MessageType
{
	KDTM_HELLO = 1,
	KDTM_WARNING = 2
};

/**
* \ingroup kdtm
* \brief kDTM types 
*/
class TypeHeader : public Header 
{
public:
	/// c-tor
	//TypeHeader ();
	TypeHeader (MessageType t);


	///\name Header serialization/deserialization
	//\{
	static TypeId GetTypeId();
	TypeId GetInstanceTypeId () const;
	uint32_t GetSerializedSize () const;
	void Serialize (Buffer::Iterator start) const;
	uint32_t Deserialize (Buffer::Iterator start);
	void Print(std::ostream &os) const;
	//\}

	/// Return Type
	MessageType Get () const
	{
		return m_type;
	}
	/// Check that type is valide
	bool IsValid () const 
	{
		return m_valid; /// FIXME like in GPSR 
	}
	bool operator== (TypeHeader const & o) const;
private:
	MessageType m_type;
	bool m_valid;
};

std::ostream & operator<< (std::ostream & os, TypeHeader const & h);

/**
* \ingroup kdtm
* \brief   Hello Message Format
  \verbatim

  \endverbatim
*/
class HelloHeader : public Header 
{
public:
	///c-tor
  HelloHeader (uint32_t id = 0, 
  						 uint64_t originPosx = 0, 
  						 uint64_t originPosy = 0,
  						 uint64_t speedx = 0,
  						 uint64_t speedy = 0,
  						 uint64_t trajectoryBegin = 0,
  						 uint64_t beta = 0);

	///\name Header serialization/deserialization
	//\{
	static TypeId GetTypeId ();
	TypeId GetInstanceTypeId () const;
	uint32_t GetSerializedSize () const;
	void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
	void Print (std::ostream &os) const;
	//\}

	///\name Fields
	//\{
	void SetOriginPosx (uint64_t posx)
	{
		m_originPosx = posx;
	}
	uint64_t GetOriginPosx () const
	{
		return m_originPosx;
	}
	void SetOriginPosy (uint64_t posy) 
	{
		m_originPosy = posy;
	}
	uint64_t GetOriginPosy () const
	{
		return m_originPosy;
	}
	void SetId (uint64_t id) 
	{
		m_id = id;
	}
	uint64_t GetId () const
	{
		return m_id;
	}
	void SetSpeedx (uint64_t speedx) 
	{
		m_speedx = speedx;
	}
	uint64_t GetSpeedx () const
	{
		return m_speedx;
	}
	void SetSpeedy (uint64_t speedy) 
	{
		m_speedy = speedy;
	}
	uint64_t GetSpeedy () const
	{
		return m_speedy;
	}
	void SetTrajectoryBegin (uint64_t trajectoryBegin) 
	{
		m_trajectoryBegin = trajectoryBegin;
	}
	uint64_t GetTrajectoryBegin () const
	{
		return m_trajectoryBegin;
	}
	void SetBeta (uint64_t beta) 
	{
		m_beta = beta;
	}
	uint64_t GetBeta () const
	{
		return m_beta;
	}
	//\}

	bool operator== (HelloHeader const & o) const;

private:

	uint32_t m_id; // id of source

	uint64_t m_originPosx;
	uint64_t m_originPosy;

	uint64_t m_speedx;
	uint64_t m_speedy;

	uint64_t m_trajectoryBegin; // time when its trajectory started
	uint64_t m_beta;  // inverse of average time of trajectory => poisson coeff of stability; 
};

/**
* \ingroup kdtm
* \brief   Warning Message Format
  \verbatim

  \endverbatim
*/
class WarningHeader : public Header 
{
public:
	/// c-tor
	WarningHeader (uint32_t sourceId = 0, 
		uint32_t m_prevHopId = 0,
		uint32_t m_hopCount = 0,
	 	uint32_t messageId = 0,
	  uint64_t positionx = 0, 
	  uint64_t positiony = 0);

	///\name Header serialization/deserialization
	//\{
	static TypeId GetTypeId ();
	TypeId GetInstanceTypeId () const;
	uint32_t GetSerializedSize () const;
	void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
	void Print (std::ostream &os) const;
	//\}

	void SetSourceId (uint32_t sourceId) 
	{
		m_sourceId =  sourceId;
	}
	uint32_t GetSourceId () const
	{
		return m_sourceId;
	}	
	void SetPrevHopId (uint32_t prevHopId) 
	{
		m_prevHopId =  prevHopId;
	}
	uint32_t GetPrevHopId () const
	{
		return m_prevHopId;
	}	
	void SetHopCount (uint32_t hopCount) 
	{
		m_hopCount =  hopCount;
	}
	uint32_t GetHopCount () const
	{
		return m_hopCount;
	}	
	void SetMessageId (uint32_t messageId) 
	{
		m_messageId = messageId;
	}
	uint32_t GetMessageId () const
	{
		return m_messageId;
	}	
	void SetPostionx (uint64_t positionx) 
	{
		m_positionx = positionx;
	}
	uint64_t GetPositionx () const
	{
		return m_positionx;
	}	
	void SetPositiony (uint64_t positiony) 
	{
		m_positiony = positiony;
	}
	uint64_t GetPositiony () const
	{
		return m_positiony;
	}	

private:
	uint32_t m_sourceId;	
	uint32_t m_prevHopId;
	uint32_t m_hopCount;
	uint32_t m_messageId;	
	uint64_t m_positionx;
	uint64_t m_positiony;
};

}
}


#endif /* KDTMPACKET_H */