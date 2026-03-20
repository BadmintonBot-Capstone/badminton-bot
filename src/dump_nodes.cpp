// dump_nodes.cpp — Enumerate every GenICam node on connected Blackfly S cameras.
// Prints name, type, access, current value, and range/options for each node.

#include <Spinnaker.h>
#include <cstdio>
#include <string>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;

static const char* access_str(EAccessMode a)
{
    switch (a) {
    case RO: return "RO";
    case WO: return "WO";
    case RW: return "RW";
    case NI: return "NI";
    case NA: return "NA";
    default: return "??";
    }
}

static void dump_node(CNodePtr node)
{
    if (!node) return;

    try {

    EAccessMode access = node->GetAccessMode();

    std::string name = node->GetName().c_str();
    std::string category = "";
    NodeList_t parents;
    node->GetParents(parents);
    if (parents.size() > 0)
        category = parents[0]->GetName().c_str();

    std::string type_str;
    std::string value_str;
    std::string range_str;

    // For NI/NA nodes we can still print their type but not read values safely.
    bool readable = (access == RO || access == RW);

    switch (node->GetPrincipalInterfaceType()) {
    case intfIInteger: {
        CIntegerPtr n = static_cast<CIntegerPtr>(node);
        type_str = "Integer";
        if (readable) {
            value_str = std::to_string(n->GetValue());
            range_str = "[" + std::to_string(n->GetMin()) + " .. " +
                        std::to_string(n->GetMax()) + "] inc=" +
                        std::to_string(n->GetInc());
        }
        break;
    }
    case intfIFloat: {
        CFloatPtr n = static_cast<CFloatPtr>(node);
        type_str = "Float";
        if (readable) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.6g", n->GetValue());
            value_str = buf;
            std::snprintf(buf, sizeof(buf), "[%.6g .. %.6g]", n->GetMin(), n->GetMax());
            range_str = buf;
        }
        break;
    }
    case intfIBoolean: {
        CBooleanPtr n = static_cast<CBooleanPtr>(node);
        type_str = "Boolean";
        if (readable)
            value_str = n->GetValue() ? "true" : "false";
        break;
    }
    case intfIString: {
        CStringPtr n = static_cast<CStringPtr>(node);
        type_str = "String";
        if (readable)
            value_str = "\"" + std::string(n->GetValue().c_str()) + "\"";
        break;
    }
    case intfIEnumeration: {
        CEnumerationPtr n = static_cast<CEnumerationPtr>(node);
        type_str = "Enum";
        if (readable) {
            auto current = n->GetCurrentEntry();
            if (current) value_str = current->GetSymbolic().c_str();
        }

        NodeList_t entries;
        n->GetEntries(entries);
        range_str = "{";
        bool first = true;
        for (auto& e : entries) {
            auto* entry = dynamic_cast<IEnumEntry*>(e);
            if (!entry) continue;
            if (!first) range_str += ", ";
            std::string sym = entry->GetSymbolic().c_str();
            if (!IsReadable(CNodePtr(e))) sym += "(N/A)";
            range_str += sym;
            first = false;
        }
        range_str += "}";
        break;
    }
    case intfICommand: {
        type_str = "Command";
        value_str = "(executable)";
        break;
    }
    default:
        type_str = "Other";
        break;
    }

    std::printf("  [%s] %-40s %-8s %-4s  value=%-20s %s\n",
                category.c_str(), name.c_str(), type_str.c_str(),
                access_str(access), value_str.c_str(), range_str.c_str());

    } catch (const Spinnaker::Exception& e) {
        std::printf("  %-40s  *** READ ERROR: %s\n", node->GetName().c_str(), e.what());
    }
}

static void dump_category(CNodePtr node, INodeMap& nodemap)
{
    CCategoryPtr cat = dynamic_cast<ICategory*>(&(*node));
    if (!cat) return;

    std::printf("\n=== %s ===\n", cat->GetName().c_str());

    FeatureList_t features;
    cat->GetFeatures(features);
    for (auto& feat : features) {
        if (feat->GetPrincipalInterfaceType() == intfICategory)
            dump_category(feat, nodemap);
        else
            dump_node(feat);
    }
}

int main()
{
    auto system = System::GetInstance();
    auto cam_list = system->GetCameras();
    unsigned int n = cam_list.GetSize();

    if (n == 0) {
        std::puts("No cameras found.");
        cam_list.Clear();
        system->ReleaseInstance();
        return 1;
    }

    std::printf("Found %u camera(s), dumping first camera only.\n", n);

    auto cam = cam_list.GetByIndex(0);
    cam->Init();

    std::string serial = cam->DeviceSerialNumber.ToString().c_str();
    std::printf("\n########## Camera 0  (serial: %s) ##########\n", serial.c_str());

    auto& nodemap = cam->GetNodeMap();
    CNodePtr root = nodemap.GetNode("Root");
    dump_category(root, nodemap);

    cam->DeInit();
    cam = nullptr;

    cam_list.Clear();
    system->ReleaseInstance();
    return 0;
}
