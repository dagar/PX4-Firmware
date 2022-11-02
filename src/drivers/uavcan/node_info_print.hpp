
#pragma once

#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/node_info_retriever.hpp>

class NodeInfoPrint : public uavcan::INodeInfoListener
{

public:

	NodeInfoPrint(uavcan::INode &node)
	{ }

	~NodeInfoPrint()
	{

	}

	/**
	 * Called when a response to GetNodeInfo request is received. This happens shortly after the node restarts or
	 * becomes online for the first time.
	 * @param node_id   Node ID of the node
	 * @param response  Node info struct
	 */
	void handleNodeInfoRetrieved(uavcan::NodeID node_id, const uavcan::protocol::GetNodeInfo::Response &node_info) override
	{
		// NodeStatus status
		int uptime_s = node_info.status.uptime_sec;

		// SoftwareVersion software_version
		//	uint8 major
		//	uint8 minor
		//
		//	uint8 OPTIONAL_FIELD_FLAG_VCS_COMMIT = 1
		//	uint8 OPTIONAL_FIELD_FLAG_IMAGE_CRC  = 2
		//	uint8 optional_field_flags

		// uint32 vcs_commit

		// uint64 image_crc

		uint8_t sw_major_version = node_info.software_version.major;
		uint8_t sw_minor_version = node_info.software_version.minor;

		unsigned vcs_commit = node_info.software_version.vcs_commit;

		// HardwareVersion hardware_version
		//	uint8 major
		//	uint8 minor
		//
		// uint8[16] unique_id
		// uint8[<=255] certificate_of_authenticity
		uint8_t hw_major_version = node_info.hardware_version.major;
		uint8_t hw_minor_version = node_info.hardware_version.minor;
		//uint8_t unique_id[16]{};
		//memcpy(unique_id, &node_info.hardware_version.unique_id[0], sizeof(unique_id));


		PX4_INFO("node id: %d, %s, uptime: %ds", node_id.get(), node_info.name.c_str(), uptime_s);
		PX4_INFO("node id: %d, HW: %d.%d, SW: %d.%d,  VCS:0x%08x", node_id.get(),  hw_major_version, hw_minor_version,
			 sw_major_version, sw_minor_version, vcs_commit);
	}

	/**
	 * Called when the retriever decides that the node does not support the GetNodeInfo service.
	 * This method will never be called if the number of attempts is unlimited.
	 */
	void handleNodeInfoUnavailable(uavcan::NodeID node_id)
	{
		PX4_ERR("node %d info unavailable", node_id.get());
	}

	/**
	 * This call is routed directly from @ref NodeStatusMonitor.
	 * Default implementation does nothing.
	 * @param event     Node status change event
	 */
	void handleNodeStatusChange(const uavcan::NodeStatusMonitor::NodeStatusChangeEvent &event) override
	{
		(void)event;
	}

};
