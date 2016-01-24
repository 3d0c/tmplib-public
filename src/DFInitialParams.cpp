#include "DFInitialParams.h"
#include "tinyxml2.h"

using namespace tinyxml2;

void LoadParamString(XMLElement	* doc, std::string name, std::string & value)
{
    XMLElement	*p_path	= nullptr;
    p_path = doc->FirstChildElement(name.c_str());
    if (!p_path) return;
    value = p_path->Attribute("value");
};

void LoadParamFloat(XMLElement * doc, std::string name, float & value)
{
    XMLElement	*p_path	= nullptr;
    p_path = doc->FirstChildElement(name.c_str());
    if (!p_path) return;
    value = p_path->FloatAttribute("value");
};

void LoadParamUnsigned(XMLElement * doc, std::string name, size_t & value)
{
    XMLElement	*p_path	= nullptr;
    p_path = doc->FirstChildElement(name.c_str());
    if (!p_path) return;
    value = p_path->IntAttribute("value");
};

DFInitialParams::DFInitialParams(void)
{
}

DFInitialParams::~DFInitialParams(void)
{
}

void DFInitialParams::Clear()
{
    initial_params.Clear();
}

TInitialParameters & DFInitialParams::GetInitialParams()
{
    return initial_params;
}

int DFInitialParams::LoadXML(std::string path)
{
    if (!path.size()) return 1;

    XMLDocument	doc;
    XMLError err = XML_NO_ERROR;
    if ((err = doc.LoadFile(path.c_str())) != 0)
        return err;

    // initial_parameters
    {
        XMLElement	*p_initial_params	= nullptr;
        p_initial_params = doc.FirstChildElement("initial_parameters");
        if (!p_initial_params) return 1;

        LoadParamString(p_initial_params, "path_to_clothes", initial_params.path_to_clothes);
        LoadParamString(p_initial_params, "path_to_cache_clothes", initial_params.path_to_cache_clothes);
        LoadParamString(p_initial_params, "path_to_mannequins", initial_params.path_to_mannequins);
        LoadParamString(p_initial_params, "path_to_animation", initial_params.path_to_animation);

        LoadParamFloat(p_initial_params, "time_limit_dress", initial_params.time_limit_dress);
        LoadParamFloat(p_initial_params, "time_limit_first_frame", initial_params.time_limit_first_frame);
        LoadParamFloat(p_initial_params, "time_limit_per_frame", initial_params.time_limit_per_frame);
        LoadParamFloat(p_initial_params, "time_limit_total", initial_params.time_limit_total);
        LoadParamUnsigned(p_initial_params, "count_iteration", initial_params.count_iteration);

        // mannequin
        {
            XMLElement	*p_mannequin	= nullptr;
            p_mannequin = p_initial_params->FirstChildElement("mannequin");
            while (p_mannequin)
            {
                std::string cur_mannequin_ID = p_mannequin->Attribute("mannequinID");
                XMLElement	*p_morph	= nullptr;
                p_morph = p_mannequin->FirstChildElement("morph");
                while (p_morph)
                {
                    ParamMorphtarget tmp_morph_param;
                    tmp_morph_param.filename = p_morph->Attribute("relative_path");
                    tmp_morph_param.type = p_morph->Attribute("type");
                    tmp_morph_param.val = p_morph->FloatAttribute("value");
                    initial_params.morphtargets_storage[cur_mannequin_ID].push_back(tmp_morph_param);
                    p_morph = p_morph->NextSiblingElement("morph");
                }
                p_mannequin = p_mannequin->NextSiblingElement("mannequin");
            }
        }

        // dress
        {
            XMLElement	*p_dress	= nullptr;
            p_dress = p_initial_params->FirstChildElement("dress");
            while (p_dress)
            {
                initial_params.gressess_filenames.push_back(p_dress->Attribute("relative_path"));
                p_dress = p_dress->NextSiblingElement("dress");
            }
        }

        // out
        {
            XMLElement	*p_out	= nullptr;
            p_out = p_initial_params->FirstChildElement("out");
            while (p_out)
            {
                initial_params.output_filenames[p_out->IntAttribute("index")] = p_out->Attribute("relative_path");
                p_out = p_out->NextSiblingElement("out");
            }
        }
    }

    // morphing_parameters
    {
        XMLElement	*p_morphing_parameters	= nullptr;
        p_morphing_parameters = doc.FirstChildElement("morphing_parameters");
        while (p_morphing_parameters)
        {
            std::string cur_mannequin_ID = p_morphing_parameters->Attribute("mannequinID");
            XMLElement	*p_morph	= nullptr;
            p_morph = p_morphing_parameters->FirstChildElement("morph");
            DFMannequin::MorphParams temp_params;
            while (p_morph)
            {
                temp_params[p_morph->Attribute("type")] = p_morph->FloatAttribute("value");
                p_morph = p_morph->NextSiblingElement("morph");
            }
            initial_params.morphparams_storage[cur_mannequin_ID].push_back(temp_params);
            p_morphing_parameters = p_morphing_parameters->NextSiblingElement("morphing_parameters");
        }
    }
    return 0;
}

void DFInitialParams::SaveXML(std::string path)
{
    if (path.size() == 0)
        return;

    XMLDocument	doc;

    XMLDeclaration * t = doc.NewDeclaration();
    doc.LinkEndChild(t);

    // initial_parameters
    {
        XMLElement	*p_generation_clothing = doc.NewElement("initial_parameters");

        XMLElement	*p_path_to_clothes = doc.NewElement("path_to_clothes");
        p_path_to_clothes->SetAttribute("value", initial_params.path_to_clothes.c_str());
        p_generation_clothing->InsertEndChild(p_path_to_clothes);

        XMLElement	*p_path_to_cache_clothes = doc.NewElement("path_to_cache_clothes");
        p_path_to_cache_clothes->SetAttribute("value", initial_params.path_to_cache_clothes.c_str());
        p_generation_clothing->InsertEndChild(p_path_to_cache_clothes);

        XMLElement	*p_path_to_base_mannequins = doc.NewElement("path_to_mannequins");
        p_path_to_base_mannequins->SetAttribute("value", initial_params.path_to_mannequins.c_str());
        p_generation_clothing->InsertEndChild(p_path_to_base_mannequins);

        XMLElement	*p_path_to_animation = doc.NewElement("path_to_animation");
        p_path_to_animation->SetAttribute("value", initial_params.path_to_animation.c_str());
        p_generation_clothing->InsertEndChild(p_path_to_animation);

        XMLElement	*p_time_limit_dress = doc.NewElement("time_limit_dress");
        p_time_limit_dress->SetAttribute("value", initial_params.time_limit_dress);
        p_generation_clothing->InsertEndChild(p_time_limit_dress);

        XMLElement	*p_time_limit_first_frame = doc.NewElement("time_limit_first_frame");
        p_time_limit_first_frame->SetAttribute("value", initial_params.time_limit_first_frame);
        p_generation_clothing->InsertEndChild(p_time_limit_first_frame);

        XMLElement	*p_time_limit_per_frame = doc.NewElement("time_limit_per_frame");
        p_time_limit_per_frame->SetAttribute("value", initial_params.time_limit_per_frame);
        p_generation_clothing->InsertEndChild(p_time_limit_per_frame);

        XMLElement	*p_time_limit_total = doc.NewElement("time_limit_total");
        p_time_limit_total->SetAttribute("value", initial_params.time_limit_total);
        p_generation_clothing->InsertEndChild(p_time_limit_total);

        XMLElement	*p_count_iteration = doc.NewElement("count_iteration");
        p_count_iteration->SetAttribute("value", static_cast<unsigned int>(initial_params.count_iteration));
        p_generation_clothing->InsertEndChild(p_count_iteration);

        // mannequin
        for (MorphtargetsDequeStorage::const_iterator cur_morphtarget = initial_params.morphtargets_storage.cbegin(); cur_morphtarget != initial_params.morphtargets_storage.cend(); cur_morphtarget++)
        {
            XMLElement	*p_mannequin = doc.NewElement("mannequin");
            p_mannequin->SetAttribute("mannequinID", cur_morphtarget->first.c_str());
            for (MorphtargetsDeque::const_iterator cit = cur_morphtarget->second.cbegin(); cit != cur_morphtarget->second.cend(); cit++)
            {
                XMLElement	*p_morph = doc.NewElement("morph");
                p_morph->SetAttribute("relative_path", cit->filename.c_str());
                p_morph->SetAttribute("type", cit->type.c_str());
                p_morph->SetAttribute("value", cit->val);
                p_mannequin->InsertEndChild(p_morph);
            }
            p_generation_clothing->InsertEndChild(p_mannequin);
        }

        doc.InsertEndChild(p_generation_clothing);
    }

    // morphing_parameters
    for (MorphParamsStorage::const_iterator cur_morphparam = initial_params.morphparams_storage.cbegin(); cur_morphparam != initial_params.morphparams_storage.cend(); cur_morphparam++)
    {
        for (MorphParamsDeque::const_iterator cit = cur_morphparam->second.cbegin(); cit != cur_morphparam->second.cend(); cit++)
        {
            XMLElement	*p_morphing_parameters = doc.NewElement("morphing_parameters");
            p_morphing_parameters->SetAttribute("mannequinID", cur_morphparam->first.c_str());
            for (DFMannequin::MorphParams::const_iterator cur_param = cit->cbegin(); cur_param != cit->cend(); cur_param++)
            {
                XMLElement	*p_morph = doc.NewElement("morph");
                p_morph->SetAttribute("type", cur_param->first.c_str());
                p_morph->SetAttribute("value", cur_param->second);
                p_morphing_parameters->InsertEndChild(p_morph);
            }
            doc.InsertEndChild(p_morphing_parameters);
        }
    }

    doc.SaveFile(path.c_str());
}

