#include "DirectionalMovementHandler.h"
#include "Events.h"
#include "Hooks.h"
#include "ModAPI.h"
#include "Papyrus.h"
#include "Settings.h"
#include "Raycast.h"
#include "API/APIManager.h"

void MessageHandler(SKSE::MessagingInterface::Message* a_msg)
{
	// Try requesting APIs at multiple steps to try to work around the SKSE messaging bug
	switch (a_msg->type) {
	case SKSE::MessagingInterface::kDataLoaded:
		APIs::RequestAPIs();
		Events::SinkEventHandlers();
		Settings::Initialize();
		Settings::ReadSettings();		
		DirectionalMovementHandler::GetSingleton()->InitCameraModsCompatibility();
		DirectionalMovementHandler::GetSingleton()->Initialize();
		break;
	case SKSE::MessagingInterface::kPostLoad:
		APIs::RequestAPIs();
		break;
	case SKSE::MessagingInterface::kPostPostLoad:
		APIs::RequestAPIs();
		break;
	case SKSE::MessagingInterface::kPreLoadGame:
		DirectionalMovementHandler::GetSingleton()->OnPreLoadGame();
		break;
	case SKSE::MessagingInterface::kPostLoadGame:
	case SKSE::MessagingInterface::kNewGame:
		APIs::RequestAPIs();
		Settings::OnPostLoadGame();
		DirectionalMovementHandler::Register();
		break;
	}
}

namespace
{
	void InitializeLog() {
	#ifndef NDEBUG
		auto sink = std::make_shared<spdlog::sinks::msvc_sink_mt>();
	#else
		auto path = logger::log_directory();
		if (!path) {
			util::report_and_fail("Failed to find standard logging directory"sv);
		}

		*path /= fmt::format("{}.log", Plugin::NAME);
		auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path->string(), true);
	#endif

	#ifndef NDEBUG
		const auto level = spdlog::level::trace;
	#else
		constexpr auto level = spdlog::level::info;
	#endif

		auto log = std::make_shared<spdlog::logger>("global"s, std::move(sink));
		log->set_level(level);
		log->flush_on(level);

		spdlog::set_default_logger(std::move(log));
		spdlog::set_pattern("%g(%#): [%^%l%$] %v"s);
		logger::info("THIS IS A MODIFIED VERSION MADE FOR THE GIANTESS (GTS) MOD.\r\nDO NOT CONTACT ERSHIN IF YOU HAVE ISSUES WITH THIS VERSION");
	}
}


SKSEPluginLoad(const SKSE::LoadInterface* a_skse) {
#ifndef NDEBUG
	while (!IsDebuggerPresent()) {
		Sleep(100);
	}
#endif
	REL::Module::reset();  // Clib-NG bug workaround

	InitializeLog();
	logger::info("{} v{}"sv, Plugin::NAME, Plugin::VERSION.string());

	SKSE::Init(a_skse);
	SKSE::AllocTrampoline(1 << 9);

	auto messaging = SKSE::GetMessagingInterface();
	if (!messaging->RegisterListener("SKSE", MessageHandler)) {
		return false;
	}

	Hooks::Install();
	Papyrus::Register();

	return true;
}

SKSEPluginInfo(
	.Version = REL::Version{ 2, 2, 6, 0 },
	.Name = Plugin::NAME,
	.Author = "Ershin, Modified by BingusEx for the GTS Mod",
	.StructCompatibility = SKSE::StructCompatibility::Independent,
	.RuntimeCompatibility = SKSE::VersionIndependence::AddressLibrary
);

extern "C" DLLEXPORT void* SKSEAPI RequestPluginAPI(const TDM_API::InterfaceVersion a_interfaceVersion)
{
	auto api = Messaging::TDMInterface::GetSingleton();

	logger::info("TrueDirectionalMovement::RequestPluginAPI called, InterfaceVersion {}", static_cast<uint8_t>(a_interfaceVersion));

	switch (a_interfaceVersion) {
	case TDM_API::InterfaceVersion::V1:
		[[fallthrough]];
	case TDM_API::InterfaceVersion::V2:
		[[fallthrough]];
	case TDM_API::InterfaceVersion::V3:
		logger::info("TrueDirectionalMovement::RequestPluginAPI returned the API singleton");
		return static_cast<void*>(api);
	}

	logger::info("TrueDirectionalMovement::RequestPluginAPI requested the wrong interface version");
	return nullptr;
}
