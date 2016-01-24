#include "DFMesh.h"
#include <ctime>
#include <fstream>
#include <sstream>
#include <string.h>
#include <climits>
#include "zlib_utils.h"
#include "Vector2d.h"

DFMesh::DFMesh()
{
    Init(0, DO_NOT_COPY);
}

DFMesh::DFMesh(const DFMesh * const init_mesh, CopyMode copy_mode)
{
    Init(init_mesh, copy_mode);
}

void DFMesh::Init(const DFMesh * const init_mesh, CopyMode copy_mode)
{
    max_find_distance_ = static_cast<DF::FloatingPointType>(1);
    max_find_distance_2_ = max_find_distance_ * max_find_distance_;
    cube_section_size_x = max_find_distance_;
    cube_section_size_y = max_find_distance_;
    cube_section_size_z = max_find_distance_;
    cube_origin_shift_x = 250;
    cube_origin_shift_y = 250;
    cube_origin_shift_z = 250;
    if (init_mesh) {
        switch (copy_mode)
        {
        case DFMesh::RESIZE_ONLY:
            vv_.resize(init_mesh->vv_.size());
            vn_.resize(init_mesh->vn_.size());
            vt_.resize(init_mesh->vt_.size());
            vf_.resize(init_mesh->vf_.size());
            groups_.clear();
            groups_.resize(init_mesh->groups_.size());
            precision = init_mesh->precision;
            break;
        case DFMesh::COPY_COORDS:
            vv_ = init_mesh->vv_;
            vn_ = init_mesh->vn_;
            vt_ = init_mesh->vt_;
            vertex_allocation_map_ = init_mesh->vertex_allocation_map_;
            precision = init_mesh->precision;
            incidence_ = init_mesh->incidence_;
            collision_links_ = init_mesh->collision_links_;
            incedences_count_ = 0;
            collisions_links_count_ = 0;
            break;
        case DFMesh::COPY_ALL:
            vv_ = init_mesh->vv_;
            vn_ = init_mesh->vn_;
            vt_ = init_mesh->vt_;
            vf_ = init_mesh->vf_;
            groups_ = init_mesh->groups_;
            vertex_allocation_map_ = init_mesh->vertex_allocation_map_;
            precision = init_mesh->precision;
            incidence_ = init_mesh->incidence_;
            collision_links_ = init_mesh->collision_links_;
            incedences_count_ = 0;
            collisions_links_count_ = 0;
            break;
        case DFMesh::COPY_FACES:
            vf_ = init_mesh->vf_;
            groups_ = init_mesh->groups_;
            incidence_ = init_mesh->incidence_;
            collision_links_ = init_mesh->collision_links_;
            incedences_count_ = 0;
            collisions_links_count_ = 0;
            break;
        case DFMesh::DO_NOT_COPY:
        default:
            Init(0, DFMesh::DO_NOT_COPY);
            break;
        }

        log = init_mesh->log;
    } else {
        vv_.clear();
        vn_.clear();
        vt_.clear();
        vf_.clear();
        groups_.clear();
        vertex_allocation_map_.clear();
        incidence_.clear();
        collision_links_.clear();
        incedences_count_ = 0;
        collisions_links_count_ = 0;
        precision = 0;
        log.Init(0);
    }
}

void DFMesh::Clear(ClearMode clear_mode)
{
    switch (clear_mode)
    {
    case DFMesh::NULL_POINTS_CLEAR_FACES:
        for (DF::PointsVector::iterator it = vv_.begin(); it < vv_.end(); it++) *it = DF::zero;
        for (DF::PointsVector::iterator it = vn_.begin(); it < vn_.end(); it++) *it = DF::zero;
        for (DF::PointsVector::iterator it = vt_.begin(); it < vt_.end(); it++) *it = DF::zero;
        vf_.clear();
        groups_.clear();
        log.Close();
        vertex_allocation_map_.clear();
        break;
    case DFMesh::NULL_POINTS:
        for (DF::PointsVector::iterator it = vv_.begin(); it < vv_.end(); it++) *it = DF::zero;
        for (DF::PointsVector::iterator it = vn_.begin(); it < vn_.end(); it++) *it = DF::zero;
        for (DF::PointsVector::iterator it = vt_.begin(); it < vt_.end(); it++) *it = DF::zero;
        vertex_allocation_map_.clear();
        break;
    case DFMesh::NULL_VERTICES:
        for (DF::PointsVector::iterator it = vv_.begin(); it < vv_.end(); it++) *it = DF::zero;
        vertex_allocation_map_.clear();
        break;
    case DFMesh::CLEAR_FACES_GROUPS:
        vf_.clear();
        groups_.clear();
        break;
    case DFMesh::CLEAR_GROUPS:
        groups_.clear();
        break;
    case DFMesh::CLOSE_LOG:
        log.Close();
        break;
    case DFMesh::CLEAR_INTERNAL:
        vertex_allocation_map_.clear();
        incidence_.clear();
        collision_links_.clear();
        incedences_count_ = 0;
        collisions_links_count_ = 0;
        normal_field_.clear();
        break;
    case CLEAR_ALL:
        vv_.clear();
        vn_.clear();
        vt_.clear();
        vf_.clear();
        groups_.clear();
        incidence_.clear();
        collision_links_.clear();
        incedences_count_ = 0;
        collisions_links_count_ = 0;
        vertex_allocation_map_.clear();  // 3ı ÏÂÌ˚È Ï‡ÒÒË‚ ÚÓ˜ÂÍ ËÌ‰ÂÍÒÓ‚ ÒÓ„Î‡ÒÌÓ Ëı ‡ÒÔÂ‰ÂÎÂÌË˛ ÔÓ ÍÛ·‡Ï (ËÌ‰ÂÍÒ ÛÏÌÓÊÂÌ Ì‡ 3)
        normal_field_.clear();  // ÔÓÎÂ ÌÓÏ‡ÎÂÈ (ÒÂ‰ÌËÂ ÌÓÏ‡ÎË ÔÓ ÍÛ·‡Ï) (ÒÓÓÚ‚ÂÚÒÚ‚ÂÌÌÓ vertex_allocation_map_)
        break;
    case DFMesh::DO_NOT_CLEAR:
    default:
        break;
    }
}

DF::Error_t DFMesh::ImportOBJ(const std::string &fn, bool bPack)
{
    log << "[" << DF::GetCurrentTimeString() << "] Importing WaveFront OBJ (" << fn << ") started\n";
    clock_t start = clock();
    std::ifstream ifs (fn.c_str(), std::ifstream::binary);

    if (!ifs.good())
    {
        log << "[" << DF::GetCurrentTimeString() << "] Importing WaveFront OBJ (" << fn << ") is impossible\n";
        return DF::err_file_io_problem;
    }

    std::filebuf* pbuf = ifs.rdbuf();
    size_t size = static_cast<size_t>(pbuf->pubseekoff (0, ifs.end, ifs.in));
    pbuf->pubseekpos (0,ifs.in);
    char* buffer=new char[size + 1];
    pbuf->sgetn (buffer,size);
    ifs.close();
    buffer[size] = 0;

    DF::Error_t err = ParseWavefrontBuffer(buffer, bPack);
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Importing WaveFront OBJ (" << fn << ") finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";

    delete[] buffer;

    return err;
}

DF::Error_t DFMesh::ExportOBJ(const std::string &fn, int out_mode, bool bPack, int compression_level) const
{
    size_t so = 0;
    char * buf = 0;

    log << "[" << DF::GetCurrentTimeString() << "] Exporting WaveFront OBJ (" << fn << ") started\n";
    clock_t start = clock();
    DF::Error_t err = ConvertToWavefrontBuffer((void**)&buf, so, (out_mode & OUT_NORMALS) > 0 ? true : false, (out_mode & OUT_TEXTURES) > 0 ? true : false, bPack, compression_level);

    if (err == DF::err_ok)
    {
        std::ofstream os(fn.c_str(), std::ifstream::binary);
        if (!os.good())
        {
            log << "Exporting WaveFront OBJ (" << fn << ") is impossible\n";
            delete[] buf;
            return DF::err_file_io_problem;
        }     
        os.write(buf, so);
        os.close();
    }
    delete[] buf;

    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Exporting WaveFront OBJ (" << fn << ") finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}

DF::Error_t DFMesh::ParseWavefrontBuffer(void *objData, size_t objSize, bool bPack)
{
    log << "[" << DF::GetCurrentTimeString() << "] Parsing Wavefront Buffer started\n";
    clock_t start = clock();

    vv_.clear();
    vn_.clear();
    vt_.clear();
    vf_.clear();
    groups_.clear();

    if (bPack)
    {
//         uint8_t* pbuf = new uint8_t[objSize];
//         try {
//             UncompressData();
//         }
//         catch (...) {
//             log << '[' << DF::GetCurrentTimeString() << "] ParseWavefrontBuffer(): An exception during data uncompression has catched\n";
//         }
    }

    // ‚˚˜ËÒÎËÏ precision ÒÌ‡ÛÊË ÓÒÌÓ‚ÌÓ„Ó ˆËÍÎ‡ Ô‡ÒËÌ„‡
    unsigned char *p_ch = static_cast<unsigned char*>(objData);
    size_t groups_size = groups_.size();
    while (*p_ch != 0)
    {
        if (p_ch[0] == 'v' && DF::isblank(p_ch[1]) || p_ch[0] == 'v' && p_ch[1] == 't' && DF::isblank(p_ch[2]) || p_ch[0] == 'v' && p_ch[1] == 'n' && DF::isblank(p_ch[2]))
        {
            do p_ch++; while (*p_ch != '.');
            unsigned char *p_ch1 = ++p_ch;
            while (isdigit(*p_ch1) && *p_ch1 != 0) p_ch1++;
            precision = static_cast<unsigned int>(p_ch1 - p_ch);
            break;
        }
        while (!iscntrl(*p_ch)) p_ch++;
        while (iscntrl(*p_ch)) p_ch++;
    }

    p_ch = static_cast<unsigned char*>(objData);
    while (*p_ch != 0)
    {
        if (*p_ch == 'v')
        {
            p_ch++;
            DF::PointsVector *p_v = &vv_;
            
            if (*p_ch == 't') { 
                p_v = &vt_; 
                p_ch++; 
            }
            else if (*p_ch == 'n') { 
                p_v = &vn_; 
                p_ch++; 
            }
            
            if (!DF::isblank(*p_ch)) {
                return DF::err_file_structure_problem;
            }

            do p_ch++; while (DF::isblank(*p_ch));

            p_v->push_back(DF::parse<DF::FloatingPointType>(&p_ch));
            
            if (*p_ch == 0) {
                return DF::err_file_structure_problem;
            }

            do p_ch++; while (DF::isblank(*p_ch));
            
            p_v->push_back(DF::parse<DF::FloatingPointType>(&p_ch));
            
            if (*p_ch == 0) {
                return DF::err_file_structure_problem;
            }

            while (DF::isblank(*p_ch)) {
                p_ch++;
            }

            p_v->push_back(DF::parse<DF::FloatingPointType>(&p_ch));
        }

        else if (*p_ch == 'f')
        {
            p_ch++;
            do p_ch++; while (!isdigit(*p_ch));

            vf_.resize(vf_.size() + 1);
            
            while (!iscntrl(*p_ch))
            {
                DF::FaceItem f;
                f.v = DF::parse<DF::Index>(&p_ch) - 1;

                if (*p_ch == '/') {
                    do p_ch++; while (!isdigit(*p_ch));
                    f.t = DF::parse<DF::Index>(&p_ch) - 1;
                    if (*p_ch == '/') {
                        do p_ch++; while (!isdigit(*p_ch));
                        f.n = DF::parse<DF::Index>(&p_ch) - 1;
                    }
                }

                vf_.back().push_back(f);
                
                while (DF::isblank(*p_ch)) p_ch++;
            }
        }

        else if (*p_ch == 'g' || *p_ch == 'u') 
        {
            groups_.resize(groups_.size() + 1);
            
            groups_.back().v_last = vv_.size() - 1;
            groups_.back().t_last = vt_.size() - 1;
            groups_.back().n_last = vn_.size() - 1;
            groups_.back().f_first = vf_.size();
            
            groups_.back().type = *p_ch;
            
            groups_size = groups_.size();

            do p_ch++; while (DF::isblank(*p_ch));

            if (iscntrl(*p_ch)) {
                return DF::err_file_structure_problem;
            }

            while (!iscntrl(*p_ch)) {
                groups_.back().name.push_back(*p_ch++);
            }
        }

        else if (*p_ch == '#')
        {
        }

        while (!iscntrl(*p_ch)) p_ch++;
        
        while (*p_ch != 0 && iscntrl(*p_ch)) p_ch++;
    }

    if (groups_size == 0)
    {
        AddStandartGroup("standart_group");
    }
    else
    {
        groups_[0].v_first = 0;
        groups_[0].t_first = 0;
        groups_[0].n_first = 0;
        for (size_t i = 1; i < groups_.size(); i++) {
            groups_[i].v_first = groups_[i - 1].v_last + 1;
            groups_[i].t_first = groups_[i - 1].t_last + 1;
            groups_[i].n_first = groups_[i - 1].n_last + 1;
        }
        for (size_t i = 0; i < groups_.size() - 1; i++) {
            groups_[i].f_last = groups_[i + 1].f_first - 1;
        }
        groups_.back().f_last = vf_.size() - 1;
        // ‚˚˜ËÒÚËÏ Ô‡‡ÁËÚÌ˚Â „ÛÔÔ˚
        /*
        size_t i = 0;
        while (i < groups_.size())
        {
            if (groups_[i].f_first > groups_[i].f_last || groups_[i].v_first > groups_[i].v_last || groups_[i].f_last == static_cast<DF::Index>(-1) || groups_[i].v_last == static_cast<DF::Index>(-1))
            else
                i++;
        }
        */
    }
   
    DF::Error_t err = DF::err_ok;

    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Parsing Wavefront Buffer finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";

    return err;
}

DF::Error_t DFMesh::ConvertToWavefrontBuffer(void **objData, size_t &objSize, bool out_normals, bool out_textures, bool bPack, int compression_level) const
{
    log << "[" << DF::GetCurrentTimeString() << "] Converting mesh to Wavefront buffer started\n";
    clock_t start = clock();

    if (vv_.empty()) return DF::err_not_enough_elements;

    size_t buf_size = 100*vv_.size()/3 + 100*vn_.size()/3 + 100*vt_.size()/3 + 150*vf_.size() + 50*groups_.size();  // „Û·‡ˇ ÓˆÂÌÍ‡ Ï‡ÍÒËÏ‡Î¸ÌÓ„Ó ‡ÁÏÂ‡ ÂÁÛÎ¸ÚËÛ˛˘Â„Ó Ù‡ÈÎ‡
    char *buf = new char [buf_size];
    char *p_ch = buf;

    DF::bufcpy(&p_ch, "# Dressformer Wavefront OBJ Exporter 2014\n");
    DF::bufcpy(&p_ch, "# File Created: ");
    DF::bufcpy(&p_ch, DF::GetCurrentTimeString().c_str());
    DF::bufcpy(&p_ch, "\n");

    for(size_t i_group = 0; i_group < groups_.size(); i_group++)
    {
        // DF::bufcpy(&p_ch, "\n#\n# object ");
        // DF::bufcpy(&p_ch, groups_[i_group].name.c_str());
        // DF::bufcpy(&p_ch, "\n#\n\n");

        const DF::FloatingPointType *first;
        const DF::FloatingPointType *last;

        if (groups_[i_group].v_first != static_cast<DF::Index>(-1) && groups_[i_group].v_last != static_cast<DF::Index>(-1))
        {
            first = &vv_[groups_[i_group].v_first];
            last = &vv_[groups_[i_group].v_last];
            for (const DF::FloatingPointType *it = first; it <= last; it += 3)
            {
                DF::bufcpy(&p_ch, "v ");
                DF::bufput(&p_ch, *it, precision);
                DF::bufcpy(&p_ch, " ");
                DF::bufput(&p_ch, *(it+1), precision);
                DF::bufcpy(&p_ch, " ");
                DF::bufput(&p_ch, *(it+2), precision);
                DF::bufcpy(&p_ch, "\n");
            }

            // DF::bufcpy(&p_ch, "# "); 
            // DF::bufput(&p_ch, static_cast<unsigned>((groups_[i_group].v_last - groups_[i_group].v_first + 1) / 3), 0);
            DF::bufcpy(&p_ch, "\n");
        }

        if (out_normals)
        {
            if (groups_[i_group].n_first != static_cast<DF::Index>(-1) && groups_[i_group].n_last != static_cast<DF::Index>(-1))
            {
                first = &vn_[groups_[i_group].n_first];
                last = &vn_[groups_[i_group].n_last];
                for (const DF::FloatingPointType *it = first; it <= last; it += 3)
                {
                    DF::bufcpy(&p_ch, "vn ");
                    DF::bufput(&p_ch, *it, precision);
                    DF::bufcpy(&p_ch, " ");
                    DF::bufput(&p_ch, *(it+1), precision);
                    DF::bufcpy(&p_ch, " ");
                    DF::bufput(&p_ch, *(it+2), precision);
                    DF::bufcpy(&p_ch, "\n");
                }
                DF::bufcpy(&p_ch, "# "); 
                DF::bufput(&p_ch, static_cast<unsigned>((groups_[i_group].n_last - groups_[i_group].n_first + 1) / 3), 0);
                DF::bufcpy(&p_ch, " vertex normals\n\n");
            }
        }

        if (out_textures)
        {
            if (groups_[i_group].t_first != static_cast<DF::Index>(-1) && groups_[i_group].t_last != static_cast<DF::Index>(-1))
            {
                first = &vt_[groups_[i_group].t_first];
                last = &vt_[groups_[i_group].t_last];
                for (const DF::FloatingPointType *it = first; it <= last; it += 3)
                {
                    DF::bufcpy(&p_ch, "vt ");
                    DF::bufput(&p_ch, *it, precision);
                    DF::bufcpy(&p_ch, " ");
                    DF::bufput(&p_ch, *(it+1), precision);
                    DF::bufcpy(&p_ch, " ");
                    DF::bufput(&p_ch, *(it+2), precision);
                    DF::bufcpy(&p_ch, "\n");
                }
                DF::bufcpy(&p_ch, "# "); 
                DF::bufput(&p_ch, static_cast<unsigned>((groups_[i_group].t_last - groups_[i_group].t_first + 1) / 3), 0);
                DF::bufcpy(&p_ch, " texture coords\n\n");
            }
        }

        if (groups_[i_group].f_first != static_cast<DF::Index>(-1) && groups_[i_group].f_last != static_cast<DF::Index>(-1))
        {
            // DF::bufcpy(&p_ch, "g ");
            DF::bufcpy(&p_ch, (const char *)&groups_[i_group].type);
            if (groups_[i_group].type == 'g') {
                DF::bufcpy(&p_ch, " ");
            }

            DF::bufcpy(&p_ch, groups_[i_group].name.c_str());
            DF::bufcpy(&p_ch, "\n");
            
            const DF::faceItemVector *ffirst = &vf_[groups_[i_group].f_first];
            const DF::faceItemVector *flast = &vf_[groups_[i_group].f_last];
            for (const DF::faceItemVector *it = ffirst; it <= flast; it++)
            {
                DF::bufcpy(&p_ch, "f ");
                for (size_t fiidx = 0; fiidx < it->size(); fiidx++)
                {
                    if ((*it)[fiidx].v != static_cast<DF::Index>(-1))
                        DF::bufput(&p_ch, static_cast<unsigned>((*it)[fiidx].v + 1), 0);
                    bool texture_index_out = out_textures && (*it)[fiidx].t != static_cast<DF::Index>(-1);
                    bool normal_index_out = out_normals && (*it)[fiidx].n != static_cast<DF::Index>(-1);
                    if (texture_index_out || normal_index_out) {
                        DF::bufcpy(&p_ch, "/");
                        if (texture_index_out)
                            DF::bufput(&p_ch, static_cast<unsigned>((*it)[fiidx].t + 1), 0);
                        if (normal_index_out) {
                            DF::bufcpy(&p_ch, "/");
                            DF::bufput(&p_ch, static_cast<unsigned>((*it)[fiidx].n + 1), 0);
                        }
                    }
                    DF::bufcpy(&p_ch, " ");
                }
                DF::bufcpy(&p_ch, "\n");
            }
            // DF::bufcpy(&p_ch, "# "); 
            // DF::bufput(&p_ch, static_cast<unsigned>(groups_[i_group].f_last - groups_[i_group].f_first + 1), 0);
            // DF::bufcpy(&p_ch, " polygons/triangles\n\n");
            // DF::bufcpy(&p_ch, "\n");
        }
    }
    // *p_ch++ = 0;

    objSize = p_ch - buf;
    *objData = new char[objSize];
    if (bPack)
    {
        try {
            size_t newObjSize = CompressData(reinterpret_cast<uint8_t*>(buf), objSize, reinterpret_cast<uint8_t*>(*objData), objSize, compression_level);
            objSize = newObjSize;
        }
        catch(...) {
            memcpy(*objData, buf, objSize);
            log << '[' << DF::GetCurrentTimeString() << "] ConvertToWavefrontBuffer(): An exception during data compression has catched\n";
        }
    }
    if (!bPack)
    {
        memcpy(*objData, buf, objSize);
    }
    delete [] buf;

    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Converting mesh to Wavefront buffer finished with status: " << DF::print_error(DF::err_ok) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)";
    log << "(" << vv_.size() << " vertices, ";
    if (out_normals)
        log << vn_.size() << " vertex normals, ";
    if (out_textures)
        log << vt_.size() << " texture coords, ";
    log << vf_.size() << " polygons/triangles)\n";

    return DF::err_ok;
}

DFMesh::~DFMesh()
{
}

DF::Error_t DFMesh::Add(const DFMesh &m, bool add_name)
{
    size_t vv_size_d3 = vv_.size() / 3;
    size_t vn_size_d3 = vn_.size() / 3;
    size_t vt_size_d3 = vt_.size() / 3;
    size_t vf_size = vf_.size();
    size_t groups_size = groups_.size();

    vv_.insert(vv_.end(), m.vv_.begin(), m.vv_.end());
    vn_.insert(vn_.end(), m.vn_.begin(), m.vn_.end());
    vt_.insert(vt_.end(), m.vt_.begin(), m.vt_.end());
    vf_.insert(vf_.end(), m.vf_.begin(), m.vf_.end());
    groups_.insert(groups_.end(), m.groups_.begin(), m.groups_.end());
//     // TODO: ÍÓÔËÓ‚‡Ú¸ springs_ Ë incidence_
//     springs_.insert(springs_.end(), m.springs_.begin(), m.springs_.end());
//     incidence_.insert(incidence_.end(), m.incidence_.begin(), m.incidence_.end());
    for (size_t i = vf_size; i < vf_.size(); i++)
    {
        for (size_t j = 0; j < vf_[i].size(); j++)
        {
            vf_[i][j].v += vv_size_d3;
            vf_[i][j].n += vn_size_d3;
            vf_[i][j].t += vt_size_d3;
        }
    }
    for (size_t i = groups_size; i < groups_.size(); i++)
    {
		if (add_name)
			groups_[i].name.push_back(static_cast<char>(i) + '0');
        groups_[i].f_first += vf_size;
        groups_[i].f_last += vf_size;
        groups_[i].v_first += vv_size_d3 * 3;
        groups_[i].v_last += vv_size_d3 * 3;
        groups_[i].t_first += vt_size_d3 * 3;
        groups_[i].t_last += vt_size_d3 * 3;
        groups_[i].n_first += vn_size_d3 * 3;
        groups_[i].n_last += vn_size_d3 * 3;
    }
    return DF::err_ok;
}

DF::Error_t DFMesh::RecalcNormals()
{
    DF::Error_t err = DF::err_ok;
    DF::PointsVector temp_vn_(vv_.size(), 0);
    vn_.resize(temp_vn_.size(), 0);
    size_t i0, i1, i2;
    DF::FloatingPointType ax, ay, az, bx, by, bz, x, y, z, L;
    for (size_t i = 0; i < vf_.size(); i++)
    {
        i0 = vf_[i][0].v * 3;
        i1 = vf_[i][1].v * 3;
        i2 = vf_[i][2].v * 3;
        ax = vv_[i0] - vv_[i1];
        ay = vv_[i0 + 1] - vv_[i1 + 1];
        az = vv_[i0 + 2] - vv_[i1 + 2];
        bx = vv_[i2] - vv_[i1];
        by = vv_[i2 + 1] - vv_[i1 + 1];
        bz = vv_[i2 + 2] - vv_[i1 + 2];
        x = az*by - ay*bz;
        y = ax*bz - az*bx;
        z = ay*bx - ax*by;
        L = sqrt(x*x + y*y + z*z);
        if (L < 1e-5) { 
            //err = DF::err_null_normal_detected;
            continue; 
        }
        L = 1 / L;
        for (size_t j = 0; j < vf_[i].size(); j++)
        {
            vf_[i][j].n = vf_[i][j].v;
            temp_vn_[vf_[i][j].v * 3] += x*L;
            temp_vn_[vf_[i][j].v * 3 + 1] += y*L;
            temp_vn_[vf_[i][j].v * 3 + 2] += z*L;
        }
    }
    // normalize
    for (size_t i = 0; i < temp_vn_.size(); i += 3)
    {
        L = sqrt(temp_vn_[i]*temp_vn_[i] + temp_vn_[i+1]*temp_vn_[i+1] + temp_vn_[i+2]*temp_vn_[i+2]);
        if (L < 1e-5) {
            //err = DF::err_null_normal_detected;
            temp_vn_[i]   = vn_[i]  ;
            temp_vn_[i+1] = vn_[i+1];
            temp_vn_[i+2] = vn_[i+2];
            if (abs(temp_vn_[i]) < 1e-1 && abs(temp_vn_[i+1]) < 1e-1 && abs(temp_vn_[i+2]) < 1e-1)
            {
                temp_vn_[i]   = 1.;
                temp_vn_[i+1] = 0.;
                temp_vn_[i+2] = 0.;
            }
            continue;
        }
        L = 1 / L;
        temp_vn_[i]   *= L;
        temp_vn_[i+1] *= L;
        temp_vn_[i+2] *= L;
    }
    for (DF::GroupsVector::iterator it = groups_.begin(); it < groups_.end(); it++)
    {
        it->n_first = it->v_first;
        it->n_last = it->v_last;
    }
    vn_.swap(temp_vn_);
    return err;
}

DF::Error_t DFMesh::SimpleFindNearestInEnvironment(DF::Index &nearest_point_index, const DFMath::Vector3 &specified_point, size_t max_counter) const
{
    nearest_point_index = static_cast<DF::Index>(-1);
    int cube_index_x = static_cast<int>((specified_point[0] + cube_origin_shift_x) / cube_section_size_x);
    int cube_index_x_begin = cube_index_x;
    int cube_index_x_end = cube_index_x;
    int cube_index_y = static_cast<int>((specified_point[1] + cube_origin_shift_y) / cube_section_size_y);
    int cube_index_y_begin = cube_index_y;
    int cube_index_y_end = cube_index_y;
    int cube_index_z = static_cast<int>((specified_point[2] + cube_origin_shift_z) / cube_section_size_z);
    int cube_index_z_begin = cube_index_z;
    int cube_index_z_end = cube_index_z;

    DF::FloatingPointType dist_2_min = 1e+8;
    size_t counter = 0;

    const DF::PointsVector &vvLink = vv_;

    do
    {
        cube_index_x_begin--;
        if (cube_index_x_begin < 0)
            break;
        cube_index_y_begin--;
        if (cube_index_y_begin < 0)
            break;
        cube_index_z_begin--;
        if (cube_index_z_begin < 0)
            break;
        cube_index_x_end++;
        cube_index_y_end++;
        cube_index_z_end++;

		int szl = static_cast<int>(vertex_allocation_map_.size());
        for (int ix = cube_index_x_begin; ix <= cube_index_x_end; ix++)
        {
            if (ix >= szl)
                break;
            DF::IndexVector3D &vertex_allocation_map_level1 = vertex_allocation_map_[ix];
			int szl1 = static_cast<int>(vertex_allocation_map_level1.size());
            for (int iy = cube_index_y_begin; iy <= cube_index_y_end; iy++)
            {
                if (iy >= szl1)
                    break;
                DF::IndexVector2D &vertex_allocation_map_level2 = vertex_allocation_map_level1[iy];
				int szl2 = static_cast<int>(vertex_allocation_map_level2.size());
                for (int iz = cube_index_z_begin; iz <= cube_index_z_end; iz++)
                {
                    if (iz >= szl2)
                        break;
                    DF::IndexVector &vertex_allocation_map_level3 = vertex_allocation_map_level2[iz];
					DF::IndexVector::const_iterator endl3 = vertex_allocation_map_level3.end();
                    for (DF::IndexVector::const_iterator iti = vertex_allocation_map_level3.begin(); iti < endl3; iti++)
                    {
						DF::Index ind = *iti;
                        DF::FloatingPointType dist_2_cur = DFMath::length2( DFMath::Vector3(vvLink[ind], vvLink[ind+1], vvLink[ind+2]) - specified_point); 
                        if (dist_2_min > dist_2_cur) {
                            dist_2_min = dist_2_cur;
                            nearest_point_index = ind;
                        }
                    }
                }
            }
        }
        counter++;
    } while (nearest_point_index == static_cast<DF::Index>(-1) && counter < max_counter);

    return nearest_point_index == static_cast<DF::Index>(-1) ? DF::err_cant_find_nearest : DF::err_ok;
}

DF::Error_t DFMesh::SimpleFindNearestInEnvironmentWithMinMargin(DF::Index &nearest_point_index_3, const DFMath::Vector3 &specified_point, const DF::FloatingPointType min_dist_2, const DF::FloatingPointType max_dist_2, size_t max_counter) const
{
    nearest_point_index_3 = static_cast<DF::Index>(-1);
    int cube_index_x = static_cast<int>((specified_point[0] + cube_origin_shift_x) / cube_section_size_x);
    int cube_index_x_begin = cube_index_x;
    int cube_index_x_end = cube_index_x;
    int cube_index_y = static_cast<int>((specified_point[1] + cube_origin_shift_y) / cube_section_size_y);
    int cube_index_y_begin = cube_index_y;
    int cube_index_y_end = cube_index_y;
    int cube_index_z = static_cast<int>((specified_point[2] + cube_origin_shift_z) / cube_section_size_z);
    int cube_index_z_begin = cube_index_z;
    int cube_index_z_end = cube_index_z;

    DF::FloatingPointType dist_2_min = 1e+8;
    size_t counter = 0;

    const DF::PointsVector &vvLink = vv_;

    do
    {
        cube_index_x_begin--;
        if (cube_index_x_begin < 0)
            break;
        cube_index_y_begin--;
        if (cube_index_y_begin < 0)
            break;
        cube_index_z_begin--;
        if (cube_index_z_begin < 0)
            break;
        cube_index_x_end++;
        cube_index_y_end++;
        cube_index_z_end++;

        int szl = static_cast<int>(vertex_allocation_map_.size());
        for (int ix = cube_index_x_begin; ix <= cube_index_x_end; ix++)
        {
            if (ix >= szl)
                break;
            DF::IndexVector3D &vertex_allocation_map_level1 = vertex_allocation_map_[ix];
            int szl1 = static_cast<int>(vertex_allocation_map_level1.size());
            for (int iy = cube_index_y_begin; iy <= cube_index_y_end; iy++)
            {
                if (iy >= szl1)
                    break;
                DF::IndexVector2D &vertex_allocation_map_level2 = vertex_allocation_map_level1[iy];
                int szl2 = static_cast<int>(vertex_allocation_map_level2.size());
                for (int iz = cube_index_z_begin; iz <= cube_index_z_end; iz++)
                {
                    if (iz >= szl2)
                        break;
                    DF::IndexVector &vertex_allocation_map_level3 = vertex_allocation_map_level2[iz];
                    DF::IndexVector::const_iterator endl3 = vertex_allocation_map_level3.end();
                    for (DF::IndexVector::const_iterator iti = vertex_allocation_map_level3.begin(); iti < endl3; iti++)
                    {
                        DF::Index ind = *iti;
                        DF::FloatingPointType dist_2_cur = DFMath::length2( DFMath::Vector3(vvLink[ind], vvLink[ind+1], vvLink[ind+2]) - specified_point); 
                        if (dist_2_min > dist_2_cur && dist_2_cur > min_dist_2 + 1e-3 && dist_2_cur <= max_dist_2)
                        {
                            dist_2_min = dist_2_cur;
                            nearest_point_index_3 = ind;
                        }
                    }
                }
            }
        }
        counter++;
    } while (nearest_point_index_3 == static_cast<DF::Index>(-1) && counter < max_counter);

    return nearest_point_index_3 == static_cast<DF::Index>(-1) ? DF::err_cant_find_nearest : DF::err_ok;
}

DF::Error_t DFMesh::SimpleFindNearestArea(DF::Index &nearest_point_index, DFMath::Vector3 &projection_point, DFMath::Vector3 &projection_normal, const DFMath::Vector3 &specified_point, size_t max_counter) const
{
    nearest_point_index = static_cast<DF::Index>(-1);
    projection_point.set(0.);
    projection_normal.set(0.);
    DFMath::Vector3 nearest_point_0(0.);

    if (SimpleFindNearestInEnvironment(nearest_point_index, specified_point, max_counter) != DF::err_ok)
        return DF::err_cant_calc_projection;

    const DF::IncedenceVector &incidence_nearest_point = incidence_[nearest_point_index / 3];
    size_t incidence_nearest_point_size = incidence_nearest_point.size();
    if (incidence_nearest_point_size < 2)
        return DF::err_cant_calc_projection;
    DF::Index nearest_point_index_1 = incidence_nearest_point[0].index;
    DF::Index nearest_point_index_2 = incidence_nearest_point[1].index;
    DF::FloatingPointType dist_2_min_1 = DFMath::length2(DFMath::Vector3(vv_[nearest_point_index_1], vv_[nearest_point_index_1 + 1], vv_[nearest_point_index_1 + 2]) - specified_point);
    DF::FloatingPointType dist_2_min_2 = DFMath::length2(DFMath::Vector3(vv_[nearest_point_index_2], vv_[nearest_point_index_2 + 1], vv_[nearest_point_index_2 + 2]) - specified_point);
    DF::sort2(nearest_point_index_1, nearest_point_index_2, dist_2_min_1, dist_2_min_2);

    for (size_t i = 2; i < incidence_nearest_point_size; i++) {
        DF::Index cur_point_index = incidence_nearest_point[i].index;
        DF::FloatingPointType dist_2_cur = DFMath::length2(DFMath::Vector3(vv_[cur_point_index], vv_[cur_point_index+1], vv_[cur_point_index+2]) - specified_point);
        if (dist_2_min_2 > dist_2_cur) {
            dist_2_min_2 = dist_2_cur;
            nearest_point_index_2 = cur_point_index;
            DF::sort2(nearest_point_index_1, nearest_point_index_2, dist_2_min_1, dist_2_min_2);
        }
    }

    //DFMath::Vector3 nearest_point_1(vv_[nearest_point_index_1], vv_[nearest_point_index_1 + 1], vv_[nearest_point_index_1 + 2]);
    //DFMath::Vector3 nearest_point_2(vv_[nearest_point_index_2], vv_[nearest_point_index_2 + 1], vv_[nearest_point_index_2 + 2]);
    DFMath::Vector3 nearest_normal_0(vn_[nearest_point_index], vn_[nearest_point_index + 1], vn_[nearest_point_index + 2]);
    DFMath::Vector3 nearest_normal_1(vn_[nearest_point_index_1], vn_[nearest_point_index_1 + 1], vn_[nearest_point_index_1 + 2]);
    DFMath::Vector3 nearest_normal_2(vn_[nearest_point_index_2], vn_[nearest_point_index_2 + 1], vn_[nearest_point_index_2 + 2]);
    // TODO: „Û·˚È ‡Ò˜∏Ú; ‰Ó‡·ÓÚ‡Ú¸ (‡ÔÔÓÍÒËÏËÓ‚‡Ú¸ ÔÓ‚ÂıÌÓÒÚˇÏË (ÒÔÎ‡ÈÌ?))
    // TODO: ÌÓÏ‡Î¸, ‚ÂÓˇÚÌÓ, ÔË‰ÂÚÒˇ ÔÂÂÒ˜ËÚ‡Ú¸
    projection_point.set(vv_[nearest_point_index], vv_[nearest_point_index + 1], vv_[nearest_point_index + 2]);
    projection_normal.set(nearest_normal_0 + nearest_normal_1 + nearest_normal_2);
    DF::FloatingPointType len = DFMath::length(projection_normal);
    if (len > 1e-3)
        projection_normal /= len;
    else
        return DF::err_null_normal_detected;
    return DF::err_ok;
}

DF::Error_t DFMesh::SimpleFindNearestAreaOriented(DF::Index &nearest_point_index_3, DFMath::Vector3 &projection_point, DFMath::Vector3 &projection_normal, const DFMath::Vector3 &specified_point, const DFMath::Vector3 &specified_normal, const DF::FloatingPointType max_dist_2) const
{
    nearest_point_index_3 = static_cast<DF::Index>(-1);
    projection_point.set(0.);
    projection_normal.set(0.);
    DFMath::Vector3 nearest_point_0(0.);
    DF::FloatingPointType dist_2_min = 0;
    bool found_inverse = false;
    do {
        if (SimpleFindNearestInEnvironmentWithMinMargin(nearest_point_index_3, specified_point, dist_2_min, max_dist_2, 1) != DF::err_ok)
            return DF::err_cant_calc_projection;

        const DF::IncedenceVector &incidence_nearest_point = incidence_[nearest_point_index_3 / 3];
        size_t incidence_nearest_point_size = incidence_nearest_point.size();
        if (incidence_nearest_point_size < 2)
            return DF::err_cant_calc_projection;
        DF::Index nearest_point_index_1 = incidence_nearest_point[0].index;
        DF::Index nearest_point_index_2 = incidence_nearest_point[1].index;
        DF::FloatingPointType dist_2_min_1 = DFMath::length2(DFMath::Vector3(vv_[nearest_point_index_1], vv_[nearest_point_index_1 + 1], vv_[nearest_point_index_1 + 2]) - specified_point);
        DF::FloatingPointType dist_2_min_2 = DFMath::length2(DFMath::Vector3(vv_[nearest_point_index_2], vv_[nearest_point_index_2 + 1], vv_[nearest_point_index_2 + 2]) - specified_point);
        DF::sort2(nearest_point_index_1, nearest_point_index_2, dist_2_min_1, dist_2_min_2);

        DF::Index cur_point_index;
        for (size_t i = 2; i < incidence_nearest_point_size; i++)
        {
            cur_point_index = incidence_nearest_point[i].index;
            DF::FloatingPointType dist_2_cur = DFMath::length2(DFMath::Vector3(vv_[cur_point_index], vv_[cur_point_index+1], vv_[cur_point_index+2]) - specified_point);
            if (dist_2_min_2 > dist_2_cur)
            {
                dist_2_min_2 = dist_2_cur;
                nearest_point_index_2 = cur_point_index;
                DF::sort2(nearest_point_index_1, nearest_point_index_2, dist_2_min_1, dist_2_min_2);
            }
        }

        //DFMath::Vector3 nearest_point_1(vv_[nearest_point_index_1], vv_[nearest_point_index_1 + 1], vv_[nearest_point_index_1 + 2]);
        //DFMath::Vector3 nearest_point_2(vv_[nearest_point_index_2], vv_[nearest_point_index_2 + 1], vv_[nearest_point_index_2 + 2]);
        DFMath::Vector3 nearest_normal_0(vn_[nearest_point_index_3], vn_[nearest_point_index_3 + 1], vn_[nearest_point_index_3 + 2]);
        DFMath::Vector3 nearest_normal_1(vn_[nearest_point_index_1], vn_[nearest_point_index_1 + 1], vn_[nearest_point_index_1 + 2]);
        DFMath::Vector3 nearest_normal_2(vn_[nearest_point_index_2], vn_[nearest_point_index_2 + 1], vn_[nearest_point_index_2 + 2]);
        // TODO: „Û·˚È ‡Ò˜∏Ú; ‰Ó‡·ÓÚ‡Ú¸ (‡ÔÔÓÍÒËÏËÓ‚‡Ú¸ ÔÓ‚ÂıÌÓÒÚˇÏË (ÒÔÎ‡ÈÌ?))
        // TODO: ÌÓÏ‡Î¸, ‚ÂÓˇÚÌÓ, ÔË‰ÂÚÒˇ ÔÂÂÒ˜ËÚ‡Ú¸
        projection_point.set(vv_[nearest_point_index_3], vv_[nearest_point_index_3 + 1], vv_[nearest_point_index_3 + 2]);
        projection_normal.set(nearest_normal_0 + nearest_normal_1 + nearest_normal_2);
        found_inverse = DFMath::ScalarProduct(specified_normal, projection_normal) < 0;
        if (found_inverse)
            dist_2_min = DFMath::length2(projection_point - specified_point);
   } while (found_inverse);
    DF::FloatingPointType len = DFMath::length(projection_normal);
    if (len > 1e-3)
        projection_normal /= len;
    else
        return DF::err_null_normal_detected;
    return DF::err_ok;
}

DF::Error_t DFMesh::GenerateAllocationMap() const
{
    vertex_allocation_map_.resize(0);
    int cube_index_x = 0;
    int cube_index_y = 0;
    int cube_index_z = 0;
    for (size_t i = 0; i < vv_.size(); i += 3)
    {
        cube_index_x = static_cast<int>((vv_[i] + cube_origin_shift_x) / cube_section_size_x);
        if (cube_index_x < 0)
            cube_index_x = 0;
        cube_index_y = static_cast<int>((vv_[i+1] + cube_origin_shift_y) / cube_section_size_y);
        if (cube_index_y < 0)
            cube_index_y = 0;
        cube_index_z = static_cast<int>((vv_[i+2] + cube_origin_shift_z) / cube_section_size_z);
        if (cube_index_z < 0)
            cube_index_z = 0;
        if (static_cast<int>(vertex_allocation_map_.size()) <= cube_index_x)
            vertex_allocation_map_.resize(cube_index_x + 1);
        if (static_cast<int>(vertex_allocation_map_[cube_index_x].size()) <= cube_index_y)
            vertex_allocation_map_[cube_index_x].resize(cube_index_y + 1);
        if (static_cast<int>(vertex_allocation_map_[cube_index_x][cube_index_y].size()) <= cube_index_z)
            vertex_allocation_map_[cube_index_x][cube_index_y].resize(cube_index_z + 1);

        vertex_allocation_map_[cube_index_x][cube_index_y][cube_index_z].push_back(i);
    }
    return DF::err_ok;
}

void DFMesh::recurse_spread_force(const DF::Index &tr_cur_point_index, const DFMath::Vector3 &cur_force, DF::Vector3Vector &forces_spread, DF::StateVector &skipping_map, const DF::IndexVector &fixed_points, const DF::IndexVector &fixed_points_inverse, const DF::StateVector &point_status, DF::FloatingPointType attenuation_coefficient, const size_t cur_level, const size_t max_level) const
{
    DF::IndexVector tr_indexes;
    DF::Vector3Vector new_forces;
    DFMath::Vector3 new_force;
    DF::FloatingPointType cur_force_length_2 = DFMath::length2(cur_force);
    if (cur_level < max_level && cur_force_length_2 > 1e-2) {
        skipping_map[tr_cur_point_index] = static_cast<DF::State>(cur_level);
        const DF::Index &point_index = fixed_points[tr_cur_point_index];
        for (size_t incidence_i = 0; incidence_i < incidence_[point_index].size(); incidence_i++)
        {
            size_t linked = incidence_[point_index][incidence_i].index / 3;
            size_t tr_linked = fixed_points_inverse[linked];
            if (point_status[linked] < 2 || skipping_map[tr_linked] <= static_cast<int>(cur_level))
                continue;
            DFMath::Vector3 v(vv_[linked * 3] - vv_[point_index * 3], vv_[linked * 3 + 1] - vv_[point_index * 3 + 1], vv_[linked * 3 + 2] - vv_[point_index * 3 + 2]);
            DF::FloatingPointType len_2 = DFMath::length2(v);
            if (len_2 > 1e-2)
                v *= DFMath::ScalarProduct(v, cur_force) / len_2 * attenuation_coefficient;
            forces_spread[tr_linked] += v;
            new_forces.push_back(v);
            tr_indexes.push_back(tr_linked);
        }
        for (size_t i = 0; i < tr_indexes.size(); i++)
            recurse_spread_force(tr_indexes[i], new_forces[i], forces_spread, skipping_map, fixed_points, fixed_points_inverse, point_status, attenuation_coefficient, cur_level + 1, max_level);
    }
}

void DFMesh::CalcInternalParams()
{
    // ‚˚˜ËÒÎÂÌËÂ Ë ÏËÌËÏ‡Î¸ÌÓÈ Ï‡ÍÒËÏ‡Î¸ÌÓÈ ‰‡Î¸ÌÓÒÚÂÈ ÔÓËÒÍ‡
    min_dist_2_ = 1e+8;
    max_dist_2_ = 0;
    for (size_t i = 0; i < incidence_.size(); i++)
    {
        for (size_t j = 0; j < incidence_[i].size(); j++)
        {
            if (min_dist_2_ > incidence_[i][j].distance)
                min_dist_2_ = incidence_[i][j].distance;
            if (max_dist_2_ < incidence_[i][j].distance)
                max_dist_2_ = incidence_[i][j].distance;
        }
    }
//     max_dist_2_ *= 2;
    min_dist_2_ *= min_dist_2_;
    max_dist_2_ *= max_dist_2_;
}

DF::Error_t DFMesh::GenerateNormalsField()
{
    log << "[" << DF::GetCurrentTimeString() << "] Generating field of normals started\n";
    clock_t start = clock();

    // Ò„ÂÌÂËÛÂÏ ‚ÂÏÂÌÌÓÂ ÔÓÎÂ ÎÓÍ‡Î¸ÌÓ ÔÓ Í‡Ê‰ÓÏÛ ÍÛ·Û
    const DF::IndexVector3D::size_type vertex_allocation_map_size = vertex_allocation_map_.size();
//     DF::Vector3Vector3D normal_field_influence(vertex_allocation_map_.size());
    DF::Vector3Vector3D &normal_field_influence = normal_field_;
    normal_field_influence.resize(vertex_allocation_map_size);
    DF::Vector3Vector3D normal_field_temp(vertex_allocation_map_size);
    DF::Vector3Vector3D mass_centers_temp(vertex_allocation_map_size);
    for (size_t ix = 0; ix < vertex_allocation_map_size; ix++)
    {
        const DF::IndexVector3D &vertex_allocation_map_level1 = vertex_allocation_map_[ix];
        const DF::IndexVector3D::size_type vertex_allocation_map_level1_size = vertex_allocation_map_level1.size();
        normal_field_influence[ix].resize(vertex_allocation_map_level1_size);
        normal_field_temp[ix].resize(vertex_allocation_map_level1_size);
        mass_centers_temp[ix].resize(vertex_allocation_map_level1_size);
        for (size_t iy = 0; iy < vertex_allocation_map_level1_size; iy++)
        {
            const DF::IndexVector2D &vertex_allocation_map_level2 = vertex_allocation_map_level1[iy];
            const DF::IndexVector2D::size_type vertex_allocation_map_level2_size = vertex_allocation_map_level2.size();
            normal_field_influence[ix][iy].resize(vertex_allocation_map_level2_size, DFMath::Vector3(static_cast<DF::FloatingPointType>(0)));
            normal_field_temp[ix][iy].resize(vertex_allocation_map_level2_size, DFMath::Vector3(static_cast<DF::FloatingPointType>(0)));
            mass_centers_temp[ix][iy].resize(vertex_allocation_map_level2_size, DFMath::Vector3(static_cast<DF::FloatingPointType>(0)));
            for (size_t iz = 0; iz < vertex_allocation_map_level2_size; iz++)
            {
                const DF::IndexVector &vertex_allocation_map_level3 = vertex_allocation_map_level2[iz];
                const DF::IndexVector::size_type vertex_allocation_map_level3_size = vertex_allocation_map_level3.size();
                DFMath::Vector3 &mass_center = mass_centers_temp[ix][iy][iz];
                DFMath::Vector3 &average_normal = normal_field_temp[ix][iy][iz];
                mass_center.set(DFMath::Vector3(static_cast<DF::FloatingPointType>(0)));
                average_normal.set(DFMath::Vector3(static_cast<DF::FloatingPointType>(0)));
                for (size_t ip = 0; ip < vertex_allocation_map_level3_size; ip++)
                {
                    const DF::Index &ind_3 = vertex_allocation_map_level3[ip];
                    mass_center += DFMath::Vector3(vv_[ind_3], vv_[ind_3+1], vv_[ind_3+2]);
                    average_normal += DFMath::Vector3(vn_[ind_3], vn_[ind_3+1], vn_[ind_3+2]);
                }
                mass_center /= static_cast<DF::FloatingPointType>(vertex_allocation_map_level3_size);
                average_normal /= static_cast<DF::FloatingPointType>(vertex_allocation_map_level3_size);

            }
        }
    }

    std::ofstream file_field("field.txt");
    // Ò„ÂÌÂËÛÂÏ ÔÓÎÂ ÍÓÏÔÓÁËˆËË ‚ÎËˇÌËˇ ÍÛ·Ó‚ ‰Û„ Ì‡ ‰Û„‡
    int margin_width = 1;
    for (int ix = margin_width; ix < static_cast<int>(normal_field_temp.size()) - margin_width; ix++) {
        for (int iy = margin_width; iy < static_cast<int>(normal_field_temp[ix].size()) - margin_width; iy++) {
            for (int iz = margin_width; iz < static_cast<int>(normal_field_temp[ix][iy].size()) - margin_width; iz++) {
                DFMath::Vector3 &cur_normal = normal_field_influence[ix][iy][iz];
                cur_normal.set(static_cast<DF::FloatingPointType>(0));
                DFMath::Vector3 cur_center(
                    cube_section_size_x * (static_cast<DF::FloatingPointType>(ix) + static_cast<DF::FloatingPointType>(0.5)) - cube_origin_shift_x,
                    cube_section_size_y * (static_cast<DF::FloatingPointType>(iy) + static_cast<DF::FloatingPointType>(0.5)) - cube_origin_shift_y,
                    cube_section_size_z * (static_cast<DF::FloatingPointType>(iz) + static_cast<DF::FloatingPointType>(0.5)) - cube_origin_shift_z);
                // ÔÓıÓ‰ ÔÓ ‚ÒÂÏ ÍÛ·‡Ï
                for (int ix_s = ix - margin_width; ix_s < ix + margin_width; ix_s++) {
                    if (ix_s < 0 || ix_s >= static_cast<int>(normal_field_temp.size()))
                        break;
                    for (int iy_s = iy - margin_width; iy_s < iy + margin_width; iy_s++) {
                        if (iy_s < 0 || iy_s >= static_cast<int>(normal_field_temp[ix_s].size()))
                            break;
                        for (int iz_s = iz - margin_width; iz_s < iz + margin_width; iz_s++) {
                            if (iz_s < 0 || iz_s >= static_cast<int>(normal_field_temp[ix_s][iy_s].size()))
                                break;
                            cur_normal += normal_field_temp[ix_s][iy_s][iz_s] / DF::sqr(DFMath::length(mass_centers_temp[ix_s][iy_s][iz_s] - cur_center) + 1);  // F = sum(Fi / (ri+1)^2), Fi - ÒÂ‰Ìˇˇ ÌÓÏ‡Î¸, ri - ‰‡Î¸ÌÓÒÚ¸ ‰Ó ÷Ã ÚÓ˜ÂÍ
                        }
                    }
                }
                file_field << "cube [" << ix << ", " << iy << ", " << iz << ", " << "]: vec = (" << cur_normal[0] << ", " << cur_normal[1] << ", " << cur_normal[2] << ")\n";
            }
        }
    }

    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Generating field of normals finished with status: " << DF::print_error(DF::err_ok) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return DF::err_ok;
}

DF::Error_t DFMesh::GetNormalFieldVector(DFMath::Vector3 &found_vector, const DFMath::Vector3 &specified_point) const
{
    int cube_index_x = static_cast<int>((specified_point[0] + cube_origin_shift_x) / cube_section_size_x);
    int cube_index_y = static_cast<int>((specified_point[1] + cube_origin_shift_y) / cube_section_size_y);
    int cube_index_z = static_cast<int>((specified_point[2] + cube_origin_shift_z) / cube_section_size_z);

// #   ifdef _DEBUG
    if (cube_index_x < 0 || cube_index_x >= static_cast<int>(normal_field_.size())) {
        log << "(" << specified_point[0] << ", " << specified_point[1] << ", " << specified_point[2] << "): " << cube_index_x << "\n";
        return DF::err_not_enough_elements;
    }
    if (cube_index_y < 0 || cube_index_y >= static_cast<int>(normal_field_[cube_index_x].size())) {
        log << "(" << specified_point[0] << ", " << specified_point[1] << ", " << specified_point[2] << "): " << cube_index_x << ", " << cube_index_y << "\n";
        return DF::err_not_enough_elements;
    }
    if (cube_index_z < 0 || cube_index_z >= static_cast<int>(normal_field_[cube_index_x][cube_index_y].size())) {
        log << "(" << specified_point[0] << ", " << specified_point[1] << ", " << specified_point[2] << "): " << cube_index_x << ", " << cube_index_y << ", " << cube_index_z << "\n";
        return DF::err_not_enough_elements;
    }
// #   endif

    found_vector.set(normal_field_[cube_index_x][cube_index_y][cube_index_z]);
//     found_vector /= DFMath::length(found_vector);

    return DF::err_ok;
}

DF::Error_t DFMesh::SmoothNormals(const size_t iterations_num, DF::PointsVector& pv)
{
    log << "[" << DF::GetCurrentTimeString() << "] Fixing normals started\n";
    clock_t start = clock();

    DF::Vector3Vector points_vec(pv.size() / 3);
    points_vec.resize(0);
    for (size_t i = 0; i < pv.size(); i += 3)
        points_vec.push_back(DFMath::Vector3(pv[i], pv[i+1], pv[i+2]));

    for (size_t i = 0; i < iterations_num; i++)
    {
        for (size_t cur_point_index = 0; cur_point_index < points_vec.size(); cur_point_index++)
        {
            DF::IncedenceVector &cur_incedence_vector = incidence_[cur_point_index];
            for (size_t linked_point_i = 0; linked_point_i < cur_incedence_vector.size(); linked_point_i++)
            {
                if (cur_incedence_vector[linked_point_i].type != DF::structural)
                    continue;
                DF::Index linked_point_index = cur_incedence_vector[linked_point_i].index / 3;
                points_vec[cur_point_index] = (points_vec[cur_point_index] + points_vec[linked_point_index]) / 2.;
                points_vec[linked_point_index] = points_vec[cur_point_index];
            }
        }
    }

    for (size_t i = 0; i < pv.size() / 3; i++) {
        DF::FloatingPointType len = DFMath::length(points_vec[i]);
        if (len < 1e-2)
//             return DF::err_null_normal_detected;
            continue;
        points_vec[i] /= len;
        pv[i*3]   = points_vec[i][0];
        pv[i*3+1] = points_vec[i][1];
        pv[i*3+2] = points_vec[i][2];
    }

    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Fixing normals finished with status: " << DF::print_error(DF::err_ok) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return DF::err_ok;
}

size_t DFMesh::GetMemoryUsage() const
{
    size_t vv_size = DF::GetMemoryUsage(vv_);
    size_t vn_size = DF::GetMemoryUsage(vn_);
    size_t vt_size = DF::GetMemoryUsage(vt_);
    size_t vf_size = DF::GetMemoryUsage(vf_);
    size_t groups_size = DF::GetMemoryUsage(groups_);
    size_t incidence_size = DF::GetMemoryUsage(incidence_);
    size_t vertex_allocation_map_size = DF::GetMemoryUsage(vertex_allocation_map_);
    size_t normal_field_size = DF::GetMemoryUsage(normal_field_);

    return sizeof(*this) + vv_size + vn_size + vt_size + vf_size + groups_size + incidence_size + vertex_allocation_map_size + normal_field_size + log.GetMemoryUsage();
}

void DFMesh::AddStandartGroup(const char *group_name)
{
    DF::GroupIdentifier group;
    group.name = std::string(group_name);
    group.f_first = 0;
    group.f_last = vf_.size() - 1;
    group.v_first = 0;
    group.v_last = vv_.size() - 1;
    group.t_first = 0;
    group.t_last = vt_.size() - 1;
    group.n_first = 0;
    group.n_last = vn_.size() - 1;
    AddGroup(group);
}

DF::Error_t DFMesh::ImportBinaryCoords(DF::PointsVector &coord_vector, BinaryType input_type) {
    switch (input_type)
    {
    case DFMesh::VERTICES:
        vv_.swap(coord_vector);
        break;
    case DFMesh::NORMALS:
        vn_.swap(coord_vector);
        break;
    case DFMesh::TEXTURES:
        vt_.swap(coord_vector);
        break;
    default:
        return DF::err_illegal_type;
        break;
    }
    return DF::err_ok;
}

DF::Error_t DFMesh::ImportBinaryFaces(DF::FacesVector &faces_vector)
{
    vf_.swap(faces_vector);
    return DF::err_ok;
}

DF::Error_t DFMesh::ExportBinaryCoords(DF::PointsVector &coord_vector, BinaryType output_type) const {
    switch (output_type)
    {
    case DFMesh::VERTICES:
        coord_vector = vv_;
        break;
    case DFMesh::NORMALS:
        coord_vector = vn_;
        break;
    case DFMesh::TEXTURES:
        coord_vector = vt_;
        break;
    default:
        return DF::err_illegal_type;
        break;
    }
    return DF::err_ok;
}

DF::Error_t DFMesh::ExportBinaryFaces(DF::FacesVector &faces_vector)
{
    faces_vector = vf_;
    return DF::err_ok;
}

void DFMesh::Release()
{
    DF::ReleaseSTLContainer(vv_);
    DF::ReleaseSTLContainer(vn_);
    DF::ReleaseSTLContainer(vt_);
    DF::ReleaseSTLContainer(vf_);
    DF::ReleaseSTLContainer(groups_);
    DF::ReleaseSTLContainer(incidence_);
    DF::ReleaseSTLContainer(vertex_allocation_map_);
    DF::ReleaseSTLContainer(normal_field_);
    log.Close();
}

DF::Error_t DFMesh::ImportBIN(const std::string &fn, bool bPack)
{
    DF::Error_t err = DF::err_ok;
    log << "[" << DF::GetCurrentTimeString() << "] Importing binary object (" << fn << ") started\n";
    clock_t start = clock();
    std::ifstream ifs (fn.c_str(), std::ifstream::binary);
    if (!ifs.good())
    {
        log << "[" << DF::GetCurrentTimeString() << "] Importing binary object (" << fn << ") is impossible\n";
        return DF::err_file_io_problem;
    }
    std::filebuf* pbuf = ifs.rdbuf();
    size_t size = static_cast<size_t>(pbuf->pubseekoff (0, ifs.end, ifs.in));
    pbuf->pubseekpos (0,ifs.in);
    char* buffer=new char[size];
    pbuf->sgetn (buffer,size);
    ifs.close();

    err = ImportBinaryArray(static_cast<void*>(buffer), bPack);
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Importing binary object (" << fn << ") finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    delete[] buffer;
    return err;
}

DF::Error_t DFMesh::ExportBIN(const std::string &fn, int out_mode, bool reform_vertices_according_textures, bool bPack, int compression_level) const
{
    DF::Error_t err = DF::err_ok;
    size_t so = 0;
    char * buf = 0;
    log << "[" << DF::GetCurrentTimeString() << "] Exporting binary object (" << fn << ") started\n";
    clock_t start = clock();
    err = ExportBinaryArray((void**)&buf, so, out_mode, reform_vertices_according_textures, bPack, compression_level);

    if (err == DF::err_ok)
    {
        std::ofstream os(fn.c_str(), std::ifstream::binary);
        if (!os.good())
        {
            log << "Exporting binary object (" << fn << ") is impossible\n";
            delete[] buf;
            return DF::err_file_io_problem;
        }     
        os.write(buf, so);
        os.close();
    }
    delete[] buf;
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Exporting binary object (" << fn << ") finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}

DF::Error_t DFMesh::ImportBinaryArray(void * const pBinData, bool bPack)
{
    BinaryHeader header;
    try
    {
        Clear(CLEAR_ALL);
        if (header.file_type_sign != *static_cast<uint64_t*>(pBinData))
            return DF::err_binary_import_error;

        header = *static_cast<BinaryHeader*>(pBinData);

        // read vertices block
        vv_.resize(0);
        if (header.block_size_vv > 0)
        {
            if (header.block_size_vv / sizeof(float) % 3 != 0)
                return DF::err_binary_import_error;
            vv_.resize(static_cast<DF::PointsVector::size_type>(header.block_size_vv / sizeof(float)));
            vv_.resize(0);
            float *block_start = static_cast<float*>(static_cast<void*>(static_cast<int8_t*>(pBinData) + header.block_position_vv));
            float *block_end = block_start + header.block_size_vv / sizeof(float);
            for (float *pbufpointerFloat = block_start; pbufpointerFloat < block_end; pbufpointerFloat++)
            {
                vv_.push_back(static_cast<DF::FloatingPointType>(*pbufpointerFloat));
            }
        }

        // read texture coors block
        vt_.resize(0);
        if (header.block_size_vt > 0)
        {
            if (header.block_size_vt / sizeof(float) % 3 != 0)
                return DF::err_binary_import_error;
            vt_.resize(static_cast<DF::PointsVector::size_type>(header.block_size_vt / sizeof(float)));
            vt_.resize(0);
            float *block_start = static_cast<float*>(static_cast<void*>(static_cast<int8_t*>(pBinData) + header.block_position_vt));
            float *block_end = block_start + header.block_size_vt / sizeof(float);
            for (float *pbufpointerFloat = block_start; pbufpointerFloat < block_end; pbufpointerFloat++)
            {
                vt_.push_back(static_cast<DF::FloatingPointType>(*pbufpointerFloat));
            }
        }

        // read normal vectors block
        vn_.resize(0);
        if (header.block_size_vn > 0)
        {
            if (header.block_size_vn / sizeof(float) % 3 != 0)
                return DF::err_binary_import_error;
            vn_.resize(static_cast<DF::PointsVector::size_type>(header.block_size_vn / sizeof(float)));
            vn_.resize(0);
            float *block_start = static_cast<float*>(static_cast<void*>(static_cast<int8_t*>(pBinData) + header.block_position_vn));
            float *block_end = block_start + header.block_size_vn / sizeof(float);
            for (float *pbufpointerFloat = block_start; pbufpointerFloat < block_end; pbufpointerFloat++)
            {
                vn_.push_back(static_cast<DF::FloatingPointType>(*pbufpointerFloat));
            }
        }

        // read faces block
        vf_.resize(0);
        if (header.block_size_vf > 0)
        {
            uint32_t *block_start = static_cast<uint32_t*>(static_cast<void*>(static_cast<int8_t*>(pBinData) + header.block_position_vf));
            uint32_t *block_end = block_start + header.block_size_vf / sizeof(uint32_t);
            uint32_t *pbufpointerINT32 = block_start;
            uint32_t faces_quantity = *pbufpointerINT32++;
            vf_.resize(faces_quantity);
            for (uint32_t i = 0; i < faces_quantity; i++)
            {
                uint32_t items_quantity = *pbufpointerINT32++;
                vf_[i].resize(items_quantity);
                for (uint32_t j = 0; j < items_quantity; j++)
                {
                    vf_[i][j].v = static_cast<DF::Index>(*pbufpointerINT32++);
                    vf_[i][j].t = static_cast<DF::Index>(*pbufpointerINT32++);
                    vf_[i][j].n = static_cast<DF::Index>(*pbufpointerINT32++);
                }
            }
            if (pbufpointerINT32 != block_end)
                return DF::err_binary_import_error;
        }

        // read groups
        if (groups_.size() == 0)
            AddStandartGroup("Material01");
    }
    catch(...)
    {
        return DF::err_binary_import_error;
    }
    return DF::err_ok;
}

DF::Error_t DFMesh::ExportBinaryArray(void **ppBinData, size_t &bufSize, int out_mode, bool reform_vertices_according_textures, bool bPack, int compression_level) const
{
    bufSize = 0;
    BinaryHeader header;
    if (reform_vertices_according_textures && vt_.size() == 0)
        reform_vertices_according_textures = false;
    try
    {
        header.block_position_vv = header.header_size;
        header.block_size_vv = (out_mode & OUT_VERTICES) ? vv_.size() * sizeof(float) : 0;
        if (reform_vertices_according_textures)
            header.block_size_vv = vt_.size() * sizeof(float);
        header.block_position_vt = header.block_position_vv + header.block_size_vv;
        header.block_size_vt = (out_mode & OUT_TEXTURES) ? vt_.size() * sizeof(float) : 0;
        header.block_position_vn = header.block_position_vt + header.block_size_vt;
        header.block_size_vn = (out_mode & OUT_NORMALS) ? vn_.size() * sizeof(float) : 0;
        header.block_position_vf = header.block_position_vn + header.block_size_vn;
        if (out_mode & OUT_FACES)
        {
            header.block_size_vf = 1;
            for (size_t i = 0; i < vf_.size(); i++)
                header.block_size_vf += 1 + vf_[i].size() * 3;
            header.block_size_vf *= sizeof(uint32_t);
        }
        else
            header.block_size_vf = 0;
        header.block_position_groups = header.block_position_vf + header.block_size_vf;
        header.block_size_groups = 0;  // TODO
        header.block_position_incidence = header.block_position_groups + header.block_size_groups;
        header.block_size_incidence = 0;  // TODO

        bufSize = static_cast<size_t>(header.block_position_incidence + header.block_size_incidence);
        *ppBinData = new int8_t[bufSize];
        *static_cast<BinaryHeader*>(*ppBinData) = header;
        float *block_float;

        if (out_mode & OUT_VERTICES)
        {
            block_float = static_cast<float*>(static_cast<void*>(static_cast<int8_t*>(*ppBinData) + header.block_position_vv));
            if (reform_vertices_according_textures)
            {
                for (DF::FacesVector::const_iterator f_it = vf_.begin(); f_it < vf_.end(); f_it++)
                {
                    for (DF::faceItemVector::const_iterator fi_it = f_it->begin(); fi_it < f_it->end(); fi_it++)
                    {
                        block_float[fi_it->t * 3] = static_cast<float>(vv_[fi_it->v * 3]);
                        block_float[fi_it->t * 3 + 1] = static_cast<float>(vv_[fi_it->v * 3 + 1]);
                        block_float[fi_it->t * 3 + 2] = static_cast<float>(vv_[fi_it->v * 3 + 2]);
                    }
                }
            }
            else
            {
                for (size_t i = 0; i < vv_.size(); i++)
                {
                    block_float[i] = static_cast<float>(vv_[i]);
                }
            }
        }

        if (out_mode & OUT_TEXTURES)
        {
            block_float = static_cast<float*>(static_cast<void*>(static_cast<int8_t*>(*ppBinData) + header.block_position_vt));
            for (size_t i = 0; i < vt_.size(); i++)
            {
                block_float[i] = static_cast<float>(vt_[i]);
            }
        }

        if (out_mode & OUT_NORMALS)
        {
            block_float = static_cast<float*>(static_cast<void*>(static_cast<int8_t*>(*ppBinData) + header.block_position_vn));
            for (size_t i = 0; i < vn_.size(); i++)
            {
                block_float[i] = static_cast<float>(vn_[i]);
            }
        }

        if (out_mode & OUT_FACES)
        {
            uint32_t *block_int32 = static_cast<uint32_t*>(static_cast<void*>(static_cast<int8_t*>(*ppBinData) + header.block_position_vf));
            *block_int32++ = static_cast<uint32_t>(vf_.size());
            for (size_t i = 0; i < vf_.size(); i++)
            {
                *block_int32++ = static_cast<uint32_t>(vf_[i].size());
                size_t face_size = vf_[i].size();
                for (size_t j = 0; j < face_size; j++)
                {
                    *block_int32++ = reform_vertices_according_textures ? static_cast<uint32_t>(vf_[i][j].t) : static_cast<uint32_t>(vf_[i][j].v);
                    *block_int32++ = static_cast<uint32_t>(vf_[i][j].t);
                    *block_int32++ = static_cast<uint32_t>(vf_[i][j].n);
                }
            }
        }
    }
    catch(...)
    {
        return DF::err_binary_export_error;
    }
    if (bPack)
    {
        uint8_t* pbuf = new uint8_t[bufSize];
        memcpy(pbuf, *ppBinData, bufSize);
        try {
            bufSize = CompressData(pbuf, bufSize, reinterpret_cast<uint8_t*>(*ppBinData), bufSize, compression_level);
        }
        catch(...) {
            memcpy(*ppBinData, pbuf, bufSize);
            log << '[' << DF::GetCurrentTimeString() << "] ExportBinaryArray(): An exception during data compression has catched\n";
        }
        delete [] pbuf;
    }
    return DF::err_ok;
}

DF::Error_t DFMesh::RecalcUVWs()
{
	if( vt_.size() == 0 )
		return DF::err_ok; // !!! ÌÂÚ ÚÂÍÒÚÛÌ˚ı ÍÓÓ‰ËÌ‡Ú
    DF::PointsVector vt(vv_.size(), static_cast<DF::FloatingPointType>(0));
    for (DF::FacesVector::iterator face_it = vf_.begin(); face_it < vf_.end(); face_it++)
    {
        for (DF::faceItemVector::iterator item_it = face_it->begin(); item_it < face_it->end(); item_it++)
        {
            memcpy(&vt[item_it->v * 3], &vt_[item_it->t * 3], sizeof(DF::FloatingPointType) * 3);
            item_it->t = item_it->v;
        }
    }
    vt_.swap(vt);
    for (DF::GroupsVector::iterator group_it = groups_.begin(); group_it < groups_.end(); group_it++)
    {
        group_it->t_first = group_it->v_first;
        group_it->t_last = group_it->v_last;
    }
    return DF::err_ok;
}

DF::Error_t DFMesh::RecalcUVWsExtended()
{
    if( vt_.size() == 0 )
        return DF::err_ok; // !!! ÌÂÚ ÚÂÍÒÚÛÌ˚ı ÍÓÓ‰ËÌ‡Ú
    DF::PointsVector vv(vt_.size(), DF::zero);
    // 
    for (DF::FacesVector::iterator face_it = vf_.begin(); face_it < vf_.end(); face_it++)
    {
        for (DF::faceItemVector::iterator item_it = face_it->begin(); item_it < face_it->end(); item_it++)
        {
            memcpy(&vv[item_it->t * 3], &vv_[item_it->v * 3], sizeof(DF::FloatingPointType) * 3);
            item_it->v = item_it->t;
        }
    }
    vv_.swap(vv);
    for (DF::GroupsVector::iterator group_it = groups_.begin(); group_it < groups_.end(); group_it++)
    {
        group_it->v_first = group_it->t_first;
        group_it->v_last = group_it->t_last;
    }
    return DF::err_ok;
}

DF::Error_t DFMesh::GenerateContinuityMap(DF::IndexVector &vv_colors, DF::IndexVector &vf_colors, DF::Index &colors_num) const
{
    vv_colors.resize(0);
    vv_colors.resize(vv_.size() / 3, static_cast<DF::Index>(0));
    vf_colors.resize(0);
    vf_colors.resize(vf_.size(), static_cast<DF::Index>(0));
    DF::Index cur_color = 0; 

    do 
    {
        DF::IndexVector::iterator start_point_iter = std::find(vv_colors.begin(), vv_colors.end(), 0);
        if (start_point_iter == vv_colors.end())
            break;
        cur_color++;
        *start_point_iter = cur_color;
        bool flag = false;
        do
        {
            flag = false;
            for (size_t cur_face_index = 0; cur_face_index < vf_.size(); cur_face_index++)
            {
                if (vf_colors[cur_face_index])
                    continue;
                const DF::faceItemVector &cur_face = vf_[cur_face_index];
                for (size_t cur_face_item_index = 0; cur_face_item_index < cur_face.size(); cur_face_item_index++)
                {
                    if (vv_colors[cur_face[cur_face_item_index].v])
                    {
                        flag = true;
                        vf_colors[cur_face_index] = cur_color;
                        for (size_t i = 0; i < cur_face.size(); i++)
                            vv_colors[cur_face[i].v] = cur_color;
                        break;
                    }
                }
            }
        } while (flag);
    } while (true);

    colors_num = cur_color;

    if (cur_color == 0)
        return DF::err_unknown;

//     DFMesh temp_mesh;
//     for (size_t color = 1; color < cur_color; color++)
//     {
//         temp_mesh.Clear(CLEAR_ALL);
//         for (size_t vfi = 0; vfi < vf_.size(); vfi++)
//         {
//             if (vf_colors[vfi] == color)
//             {
//                 temp_mesh.vf_.resize(temp_mesh.vf_.size() + 1);
//                 for (size_t fitem = 0; fitem < vf_[vfi].size(); fitem++)
//                 {
//                     temp_mesh
//                 }
//             }
//         }
//         temp_mesh.RecalcNormals();
//         temp_mesh.groups_.resize(1);
//         temp_mesh.groups_[0].name = "Material_part_";
//         temp_mesh.groups_[0].name.push_back(char('0'+color/10));
//         temp_mesh.groups_[0].name.push_back(char('0'+color%10));
//         temp_mesh.groups_[0].f_first = 0;
//         temp_mesh.groups_[0].f_last = vf_.size() - 1;
//         temp_mesh.groups_[0].v_first = 0;
//         temp_mesh.groups_[0].v_last = vv_.size() ???/ 3 - 1;
//         temp_mesh.groups_[0].t_first = 0;
//         temp_mesh.groups_[0].t_last = vt_.size() ???/ 3 - 1;
//         temp_mesh.groups_[0].n_first = 0;
//         temp_mesh.groups_[0].n_last = vn_.size() ???/ 3 - 1;
//         temp_mesh.ExportWavefront(temp_mesh.groups_[0].name + std::string(".obj"));
//         meshes->push_back(temp_mesh);
//     }

    return DF::err_ok;
}

DF::Error_t DFMesh::GenerateIncidenceStruct(bool generate_stiffness_ribs, bool generate_nexus_ribs, bool b_evenly, float len_rib)
{
    incidence_.clear();
    incidence_.resize(vv_.size() / 3);
    size_t vf_size = vf_.size();
    size_t vf_item_size;
    size_t index1;
    DF::IncedenceRecord incedence_record;
    incedence_record.type = DF::structural;
    for (size_t vf_i = 0; vf_i < vf_size; vf_i++)
    {
        DF::faceItemVector &cur_face_item = vf_[vf_i];
        vf_item_size = cur_face_item.size();
        for (size_t vf_item_i = 0; vf_item_i < vf_item_size; vf_item_i++)
        {
            index1 = (vf_item_i > 0) ? vf_item_i - 1 : vf_item_size - 1;
            incedence_record.index = cur_face_item[index1].v * 3;
            incidence_[cur_face_item[vf_item_i].v].push_back(incedence_record);
            incedence_record.index = cur_face_item[vf_item_i].v * 3;
            incidence_[cur_face_item[index1].v].push_back(incedence_record);
        }
    }

	// ·ÂÁ ˝ÚËı ∏·Â Ì‡˜ËÌ‡ÂÚ ÒËÎ¸ÌÓ ÚˇÌÛÚ¸ ÚÍ‡Ì¸
    if (generate_nexus_ribs)
	// generate_nexus_ribs - ˝ÚË ∏·‡ ÔÓ¯Ë‚‡˛Ú ÌÂÒ¯ËÚ˚Â ˝ÎÂÏÂÌÚ˚, Í‡Í ÒÚ∏„‡ÌÓÂ Ó‰ÂˇÎÓ
    {
        // Ò„ÂÌÂËÛÂÏ ‰ÓÔÓÎÌËÚÂÎ¸Ì˚Â Ò‚ˇÁË (Ò¯Ë‚‡ÂÏ ÌÂÒ¯ËÚ˚Â ˝ÎÂÏÂÌÚ˚)
        incedence_record.type = DF::structural;
        CalcInternalParams();
        DF::IndexVector vv_colors;
        DF::IndexVector vf_colors;
        DF::Index colors_num = 0;
        GenerateContinuityMap(vv_colors, vf_colors, colors_num);
        if (colors_num > 1)
        {
            DF::Index nearest_point_index = static_cast<DF::Index>(-1);
            DFMath::Vector3 specified_point(static_cast<DF::FloatingPointType>(0.));
            size_t add_arcs_counter = 0;
            for (size_t point_index = 0; point_index < vv_.size() / 3; point_index++)
            {
                specified_point.set(vv_[point_index * 3], vv_[point_index * 3 + 1], vv_[point_index * 3 + 2]);
                if (SimpleFindNearestInEnvironmentWithMinMargin(nearest_point_index, specified_point, static_cast<DF::FloatingPointType>(0.), max_find_distance_2_, 1) == DF::err_ok &&
                    vv_colors[nearest_point_index / 3] != vv_colors[point_index])
                {
                    // Ò¯Ë‚‡ÂÏ Ò Ì‡È‰ÂÌÌÓÈ ÚÓ˜ÍÓÈ
                    incedence_record.index = nearest_point_index;
                    incidence_[point_index].push_back(incedence_record);
                    incedence_record.index = point_index * 3;
                    incidence_[nearest_point_index / 3].push_back(incedence_record);
                    add_arcs_counter++;
                }
            }
        }
    }

    // Û‰‡ÎËÏ ÔÓ‚ÚÓ˚
    for (DF::IncedenceVector2D::iterator it = incidence_.begin(); it < incidence_.end(); it++)
    {
        std::sort( it->begin(), it->end() );
        it->erase( std::unique( it->begin(), it->end() ), it->end() );
    }

    if (generate_stiffness_ribs)
    {
        // Ò„ÂÌÂËÛÂÏ ‰ÓÔÓÎÌËÚÂÎ¸Ì˚Â Ò‚ˇÁË (∏·‡ ÊÂÒÚÍÓÒÚË)
        DF::IncedenceVector2D temp_incidence_(vv_.size() / 3);
        for (size_t i_point = 0; i_point < incidence_.size(); i_point++) {
            for (DF::IncedenceVector::const_iterator it_linked_point = incidence_[i_point].begin(); it_linked_point < incidence_[i_point].end(); it_linked_point++) {
                temp_incidence_[i_point].insert(temp_incidence_[i_point].end(), incidence_[it_linked_point->index / 3].begin(), incidence_[it_linked_point->index / 3].end());
            }
        }
        // Û‰‡ÎËÏ ÔÓ‚ÚÓ˚ Ë ÔÂÚÎË
        for (DF::Index i = 0; i < temp_incidence_.size(); i++) {
            DF::IncedenceVector &incdc = temp_incidence_[i];
            std::sort( incdc.begin(), incdc.end() );
            incdc.erase( std::unique( incdc.begin(), incdc.end() ), incdc.end() );
            do {
                DF::IncedenceVector::iterator it = std::find(incdc.begin(), incdc.end(), i);
                if (it == incdc.end())
                    break;
                incdc.erase(it);
            } while (true);
            for (DF::IncedenceVector::iterator temp_incidence_iterator = incdc.begin(); temp_incidence_iterator < incdc.end(); temp_incidence_iterator++)
                temp_incidence_iterator->type = DF::stiffness;
            incidence_[i].insert(incidence_[i].end(), incdc.begin(), incdc.end());
        }
    }

    // ÔÓ‰Ò˜ËÚ‡ÂÏ ‰ÎËÌ˚ ∏·Â, ÓÚÒÓÚËÛÂÏ
    incedences_count_ = 0;
    for (DF::IncedenceVector2D::iterator incedence_vec_it = incidence_.begin(); incedence_vec_it < incidence_.end(); incedence_vec_it++)
    {
        DF::Index cur_point_index_3 = (incedence_vec_it - incidence_.begin()) * 3;
        std::sort( incedence_vec_it->begin(), incedence_vec_it->end() );
        incedence_vec_it->erase( std::unique( incedence_vec_it->begin(), incedence_vec_it->end() ), incedence_vec_it->end() );
        for (DF::IncedenceVector::iterator incedence_record_it = incedence_vec_it->begin(); incedence_record_it < incedence_vec_it->end(); incedence_record_it++)
        {
            // TODO: Ò‰ÂÎ‡Ú¸ ‚˚˜ËÒÎÂÌËÂ Í‡Ê‰ÓÈ ‰ÎËÌ˚ ÔÓ 1 ‡ÁÛ
			if (!b_evenly)
			{
				incedence_record_it->distance = DFMath::length(
                DFMath::Vector3(vv_[incedence_record_it->index], vv_[incedence_record_it->index + 1], vv_[incedence_record_it->index + 2]) -
                DFMath::Vector3(vv_[cur_point_index_3], vv_[cur_point_index_3 + 1], vv_[cur_point_index_3 + 2]));
			}
			else
            {
                incedence_record_it->distance = len_rib;
            }
            ++incedences_count_;
        }
    }

    return DF::err_ok;
}

DF::Error_t DFMesh::GenerateCollisionalLinks()
{
//!!!    return DF::err_ok;
    // Ò„ÂÌÂÌËÛÂÏ Â·‡ ÛÔÂÊ‰ÂÌËˇ ÒÚÓÎÍÌÓ‚ÂÌËˇ
    // ÛÒÎÓ‚ËÂ ÒÓÁ‰‡ÌËˇ Â·‡:
    // - ÚÓ˜ÍË ·ÎËÁÍÓ (1)
    // - ÚÓ˜ÍË ÌÂ Ò‚ˇÁ‡Ì˚ Ó·˘ËÏ Â·ÓÏ (2)
    // - ‚ „‡ÙÂ Ì‡ıÓ‰ˇÚÒˇ ‰ÓÒÚ‡ÚÓ˜ÌÓ ‰‡ÎÂÍÓ ‰Û„ ÓÚ ‰Û„‡ (Í‡Í ‚‡Ë‡ÌÚ - ‚ ‡ÁÌ˚ı „ÛÔÔ‡ı) (2')
    // - ÒÍ‡ˇÌÓÂ ÔÓËÁ‚Â‰ÂÌËÂ ‚ÂÍÚÓÓ‚ Â·‡ Ë ÌÓÏ‡ÎË ËÁ ÚÓ˜ÍË-ÓÒÌÓ‚‡ÌËˇ Â„Ó > 0 (3)
    const DF::PointsVector &vvLink = vv_;
    const DF::PointsVector &vnLink = vn_;
    size_t vv_size_3 = vvLink.size() / 3;
    collision_links_.clear();
    collision_links_.resize(vv_size_3);
    collisions_links_count_ = 0;
    DF::IncedenceRecord incedence_record;
    incedence_record.type = DF::collisional;
    DFMath::Vector3 specified_point(DF::zero);
    DFMath::Vector3 specified_normal(DF::zero);
    DFMath::Vector3 close_point(DF::zero);
    DFMath::Vector3 close_normal(DF::zero);
    DFMath::Vector3 new_rib(DF::zero);
    for (size_t point_index = 0; point_index < vv_size_3; point_index++)
    {
        size_t point_index_3 = point_index * 3;
        int cube_index_x = static_cast<int>((vvLink[point_index_3] + cube_origin_shift_x) / cube_section_size_x);
        int cube_index_x_begin = cube_index_x;
        int cube_index_x_end = cube_index_x;
        int cube_index_y = static_cast<int>((vvLink[point_index_3 + 1] + cube_origin_shift_y) / cube_section_size_y);
        int cube_index_y_begin = cube_index_y;
        int cube_index_y_end = cube_index_y;
        int cube_index_z = static_cast<int>((vvLink[point_index_3 + 2] + cube_origin_shift_z) / cube_section_size_z);
        int cube_index_z_begin = cube_index_z;
        int cube_index_z_end = cube_index_z;

        DF::FloatingPointType dist_2_min = 1e+8;

        const DF::IncedenceVector &cur_incedence_vec = incidence_[point_index];
        DF::IncedenceVector &cur_collision_vec = collision_links_[point_index];

        int counter = 1;
        int max_counter = 2;;
        do
        {
            cube_index_x_begin--;
            if (cube_index_x_begin < 0)
                break;
            cube_index_y_begin--;
            if (cube_index_y_begin < 0)
                break;
            cube_index_z_begin--;
            if (cube_index_z_begin < 0)
                break;
            cube_index_x_end++;
            cube_index_y_end++;
            cube_index_z_end++;

            int szl = static_cast<int>(vertex_allocation_map_.size());
            for (int ix = cube_index_x_begin; ix <= cube_index_x_end; ix++)
            {
                if (ix >= szl)
                    break;
                DF::IndexVector3D &vertex_allocation_map_level1 = vertex_allocation_map_[ix];
                int szl1 = static_cast<int>(vertex_allocation_map_level1.size());
                for (int iy = cube_index_y_begin; iy <= cube_index_y_end; iy++)
                {
                    if (iy >= szl1)
                        break;
                    DF::IndexVector2D &vertex_allocation_map_level2 = vertex_allocation_map_level1[iy];
                    int szl2 = static_cast<int>(vertex_allocation_map_level2.size());
                    for (int iz = cube_index_z_begin; iz <= cube_index_z_end; iz++)
                    {
                        if (iz >= szl2)
                            break;
                        DF::IndexVector &vertex_allocation_map_level3 = vertex_allocation_map_level2[iz];
                        DF::IndexVector::const_iterator endl3 = vertex_allocation_map_level3.end();
                        for (DF::IndexVector::const_iterator iti = vertex_allocation_map_level3.begin(); iti < endl3; iti++)
                        {
                            // (1)
                            DF::Index ind_3 = *iti;
                            for (DF::IncedenceVector::const_iterator cit_incedence = cur_incedence_vec.begin(); cit_incedence < cur_incedence_vec.end(); cit_incedence++)
                                if (cit_incedence->index == ind_3)
                                    goto end_of_outer_loop;
                            // (2)
//                             if (GetGroupIndex3(point_index_3) == GetGroupIndex3(ind_3))
//                                 goto end_of_outer_loop;
//                             // (2')
                            specified_point.set(vvLink[point_index_3], vvLink[point_index_3+1], vvLink[point_index_3+2]);
                            specified_normal.set(vnLink[point_index_3], vnLink[point_index_3+1], vnLink[point_index_3+2]);
                            close_point.set(vvLink[ind_3], vvLink[ind_3+1], vvLink[ind_3+2]);
                            close_normal.set(vnLink[ind_3], vnLink[ind_3+1], vnLink[ind_3+2]);
                            new_rib.set(close_point - specified_point);
                            if (
                                (incedence_record.distance = DFMath::length(new_rib)) < 1/*2*/ //const!  // Ï‡ÒÍËÏ‡Î¸ÌÓÂ ÍÓÌÚÓÎËÛÂÏÓÂ ÔÂÂÏÂ˘ÂÌËÂ Ò ÔÓÏÓ˘¸˛ MovePoint() = 1
                                && DFMath::ScalarProduct(specified_normal, DFMath::normalize(new_rib)) > 0
//                                 ‰Ó·‡‚ËÚ¸ ÛÒÎÓ‚ËÂ, ÔÓ‚Âˇ˛˘ÂÂ ‰‡Î¸ÌÓÒÚ¸ ÔÓ „‡ÙÛ
                                )
                            {
                                // (3)
                                // ÒÓÁ‰‡∏Ï Â·Ó
                                incedence_record.index = ind_3;
                                cur_collision_vec.push_back(incedence_record);
                                collisions_links_count_++;
                            }
                            end_of_outer_loop:;
                        }
                    }
                }
            }
            counter++;
        } while (counter < max_counter);
        std::sort( cur_collision_vec.begin(), cur_collision_vec.end() );
        cur_collision_vec.erase( std::unique( cur_collision_vec.begin(), cur_collision_vec.end() ), cur_collision_vec.end() );
    }

    return DF::err_ok;
}

bool DFMesh::DetectPenetration(DF::FloatingPointType &scalar_correction, DFMath::Vector3 &vec, const DFMath::Vector3 &specified_point) const
{
    scalar_correction = 0;
    DF::Index nearest_point_index = static_cast<DF::Index>(-1);
    DFMath::Vector3 nearest_point(0.);
    vec.set(0.);
    if (SimpleFindNearestArea(nearest_point_index, nearest_point, vec, specified_point, 1) == DF::err_ok) 
    {
        DFMath::Vector3 radius_vect(nearest_point - specified_point);
        scalar_correction = DFMath::ScalarProduct(vec, radius_vect);
//         DF::FloatingPointType len_2 = DFMath::length2(radius_vect);
    } 
    else 
        return false;
    return true;
}

bool DFMesh::DetectPenetrationNew(DF::FloatingPointType &scalar_correction, DFMath::Vector3 &vec, const DFMath::Vector3 &specified_point) const
{
    scalar_correction = 0;
    DF::Index nearest_point_index = static_cast<DF::Index>(-1);
    DFMath::Vector3 nearest_point(0.);
    DFMath::Vector3 vv;
    if (SimpleFindNearestArea(nearest_point_index, nearest_point, vv, specified_point, 1) == DF::err_ok) 
    {
        DFMath::Vector3 radius_vect(nearest_point - specified_point);
        scalar_correction = DFMath::ScalarProduct(vec, radius_vect);
        //         DF::FloatingPointType len_2 = DFMath::length2(radius_vect);
    } 
    else 
        return false;
    return true;
}

bool DFMesh::DetectPenetrationWithMinRadius(DF::FloatingPointType &scalar_correction, DFMath::Vector3 &vec, const DFMath::Vector3 &specified_point, const DFMath::Vector3 &specified_normal, const DF::FloatingPointType max_dist_2) const
{
    scalar_correction = 0;
    DF::Index nearest_point_index = static_cast<DF::Index>(-1);
    DFMath::Vector3 nearest_point(0.);
    vec.set(0.);
    if (SimpleFindNearestAreaOriented(nearest_point_index, nearest_point, vec, specified_point, specified_normal, max_dist_2) == DF::err_ok) 
    {
        DFMath::Vector3 radius_vect(nearest_point - specified_point);
        scalar_correction = DFMath::ScalarProduct(vec, radius_vect);
        //         DF::FloatingPointType len_2 = DFMath::length2(radius_vect);
    } 
    else 
        return false;
    return true;
}

DF::Index DFMesh::GetGroupByPoint(DF::Index ind_3) const
{
    DF::GroupsVector::const_iterator git = groups_.begin();
    while (git != groups_.end() && git->v_last < ind_3)
        git++;
    return git - groups_.begin();
}

//
void DFMesh::AddVertex(const DF::FloatingPointType & vv)
{
	vv_.push_back(vv);
}

// ¬ÒÚ‡‚ËÚ¸ ‚Â¯ËÌÛ
void DFMesh::InsertVertex(size_t num, const DF::FloatingPointType & val)
{
	vv_.insert(vv_.begin() + num, val);
}

//
void DFMesh::AddCoordTexture(const DF::FloatingPointType & u, const DF::FloatingPointType & v, const DF::FloatingPointType & w)
{
	vt_.push_back(u);
	vt_.push_back(v);
	vt_.push_back(w);
}

//
void DFMesh::AddFaces(const DF::FacesVector & vf)
{
	for (auto it : vf)
		vf_.push_back(it);
}

//
void DFMesh::AddFace(const DF::faceItemVector & face)
{
	vf_.push_back(face);
}

// ¬ÒÚ‡‚ËÚ¸ ÔÎÓÒÍÓÒÚ¸
void DFMesh::InsertFace(size_t num, const DF::faceItemVector & face)
{
	vf_.insert(vf_.begin() + num, face);
}

//
bool DFMesh::FindNumFace(std::vector<size_t> & num_vert, size_t & num_face)
{
	for (DF::FacesVector::iterator it_f = vf_.begin(); it_f != vf_.end(); ++it_f)
	{
		size_t count = 0;
		for (std::vector<DF::FaceItem>::iterator it = it_f->begin(); it != it_f->end(); ++it)
		{
			auto it_p = std::find_if(num_vert.begin(), num_vert.end(), [&] (size_t & num) {return it->v == num;});
			
			if (it_p == num_vert.end())
				continue;
			
			++count;
		}

		if (count != num_vert.size())
			continue;

		num_face = std::distance(vf_.begin(), it_f);
		return true;
	}

	return false;
}

//
void DFMesh::SetFace(size_t num, const DF::faceItemVector & face)
{
	if (num >= vf_.size())
		return;

	vf_[num] = face;
}

//
void DFMesh::AddGroup(const DF::GroupIdentifier & group)
{
	groups_.push_back(group);
}

//
size_t DFMesh::CountVertex()
{
	return vv_.size();
}

// ¬ÓÁ‚‡˘‡ÂÚ ÍÓÎ-‚Ó ÚÂÍÒÚÛÌ˚ı ÍÓÓ‰ËÌ‡Ú
size_t DFMesh::CountCoordTexture()
{
	return vt_.size();
}

//
void DFMesh::SetCoordTexture(const size_t & num, DF::FloatingPointType value)
{
	if (num > vt_.size())
		return;

	vt_[num] = value;
}

//
DF::FloatingPointType DFMesh::GetCoordTexture(const size_t & num)
{
	return vt_[num];
}

//
void DFMesh::SumCoordTexture(const DF::FloatingPointType & u, const DF::FloatingPointType & v, const DF::FloatingPointType & w)
{
	for (size_t i = 0; i < vt_.size();)
	{
		vt_[i] += u; ++i;
		vt_[i] += v; ++i;
		vt_[i] += w; ++i;
	}
}

//
void DFMesh::DivisionCoordTexture(const DF::FloatingPointType & u, const DF::FloatingPointType & v, const DF::FloatingPointType & w)
{
	for (size_t i = 0; i < vt_.size();)
	{
		vt_[i] /= u; ++i;
		vt_[i] /= v; ++i;
		vt_[i] /= w; ++i;
	}
}

//
void DFMesh::MultiplyCoordTexture(const DF::FloatingPointType & u, const DF::FloatingPointType & v, const DF::FloatingPointType & w)
{
	for (size_t i = 0; i < vt_.size();)
	{
		vt_[i] *= u; ++i;
		vt_[i] *= v; ++i;
		vt_[i] *= w; ++i;
	}
}

//
void DFMesh::AddIncidence(size_t num, DF::IncedenceRecord & incid)
{
	if (num >= incidence_.size())
		return;

	incidence_[num].push_back(incid);
}

//
DF::IncedenceVector DFMesh::GetIncidence(size_t num)
{
	return incidence_.at(num);
}

//
size_t DFMesh::CountIncidence()
{
	return incidence_.size();
}

//
size_t DFMesh::CountRibs()
{
	size_t count_ribs = 0;
	//std::for_each(incidence_.begin(), incidence_.end(), [&count_ribs] (DF::IncedenceVector & c) { count_ribs += c.size(); });
	for (auto & it_f : vf_) {
		count_ribs += it_f.size();
	}
	return count_ribs;
}

//
size_t DFMesh::CountFaces()
{
	return vf_.size();
}

//
size_t DFMesh::CountGroups()
{
	return groups_.size();
}

//
DF::FloatingPointType & DFMesh::GetVertex(size_t num)
{
	return vv_[num];
}

//
bool DFMesh::NumVertexWithMinimumDistanceFromGivenVertex(DFMath::Vector3 & vec, double & min_dist, size_t & num_min_point)
{
	if (CountVertex() < 3)
		return false;

    min_dist = std::sqrt(std::abs((vv_[0]  - vec.at(0)) * (vv_[0]  - vec.at(0))  + (vv_[1] - vec.at(1)) * (vv_[1] - vec.at(1)) + (vv_[2] - vec.at(2)) * (vv_[2] - vec.at(2))));
	num_min_point = 0;
	double dist1 = 0;
	for (size_t i = 3; i < vv_.size(); i+=3)
	{
        dist1 = std::sqrt(std::abs((vv_[i]  - vec.at(0)) * (vv_[i]  - vec.at(0))  + (vv_[i + 1] - vec.at(1)) * (vv_[i + 1] - vec.at(1)) + (vv_[i + 2] - vec.at(2)) * (vv_[i + 2] - vec.at(2))));
		if ( dist1 < min_dist)
		{
			min_dist = dist1;
			num_min_point = i;
		}
	}

	return true;
}

//
void DFMesh::SetVertex(size_t num, DF::FloatingPointType value)
{
	if (num >= vv_.size())
		return;

	vv_[num] = value;
}

// ”ÒÚ‡ÌÓ‚ËÚ¸ „ÛÔÔÛ
void DFMesh::SetGroup(size_t num, DF::GroupIdentifier & group)
{
	if (num >= groups_.size())
		return;

	groups_[num] = group;
}

//
DF::GroupIdentifier & DFMesh::GetGroup(const size_t num)
{
	return groups_[num];
}

bool DFMesh::GetGroup(const std::string & name1, DF::GroupIdentifier & group)
{
	bool ret_value = false;
	auto it_g = std::find_if(groups_.begin(), groups_.end(), [name1] (DF::GroupIdentifier & _group) { return name1 == _group.name;});
	
	if (it_g != groups_.end())
	{
		group = *it_g;
		ret_value	= true;
	}
	
	return ret_value;
}

//
void DFMesh::UnionGroup()
{
	DF::GroupIdentifier union_group;
	union_group.f_first = 0;
	union_group.f_last	= vf_.size() > 0 ? vf_.size() - 1 : 0;
	union_group.v_first	= 0;
	union_group.v_last	= vv_.size() > 0 ? vv_.size() - 1 : 0;
	union_group.t_first	= 0;
	union_group.t_last	= vt_.size() > 0 ? vt_.size() - 1 : 0;
	union_group.n_first	= static_cast<DF::Index>(-1);
	union_group.n_last	= vn_.size() > 0 ? vn_.size() - 1 : static_cast<DF::Index>(-1);

	//std::for_each(groups_.begin(), groups_.end(), [&union_group] (DF::GroupIdentifier & gr) {union_group.name += gr.name;});
	union_group.name = "union_group";

	groups_.clear();
	groups_.push_back(union_group);
}

//
DF::FloatingPointType DFMesh::GetNormalize(size_t num)
{
	return vn_[num];
}
//
void DFMesh::SetNormalize(size_t num, DF::FloatingPointType nm)
{
	if (num >= vn_.size())
		return;

	vn_[num] = nm;
}

//
void DFMesh::DelVertex(size_t num, bool b_vf, size_t new_num, bool tt)
{
	// ËÌ‰ÂÍÒ ‚Â¯ËÌ Ò˜ËÚ‡ÂÚÒˇ Ò 0

	// ≈ÒÎË Û‰‡ÎˇÂÏ‡ˇ ‚Â¯ËÌ‡ ·ÓÎ¸¯Â, ÍÓÎ-‚‡ Â¯ËÌ, ÚÓ„‰‡ ÌË˜Â„Ó ‰ÂÎ‡Ú¸ ÌÂ Ì‡‰Ó, ‚˚ıÓ‰ËÏ
	if (vv_.size() < num / 3)
		return;

	// Û‰‡ÎÂÌËÂ ÍÓÓ‰ËÌ‡Ú ‚Â¯ËÌ˚
	vv_.erase(vv_.begin() + num * 3);
	vv_.erase(vv_.begin() + num * 3);
	vv_.erase(vv_.begin() + num * 3);

	size_t count_del_face = 0;	// ÍÓÎ-‚Ó Û‰‡ÎÂÌÌ˚ı ÔÓ‚ÂıÌÓÒÚÂÈ

	// ÂÒÎË ‚ vf_[n] ı‡ÌËÚ Û‰‡ÎÂÌÌ˚È ËÌ‰ÂÍÒ, Û‰‡ÎËÚ¸ ˝ÚÓÚ vf_
	// ÛÏÂÌ¸¯ËÚ¸ ËÌ‰ÂÍÒ ‚Â¯ËÌ ‚ vf_ Û ÍÓÚÓ˚ı ·ÓÎ¸¯Â num

	if (vf_.size() == 0)
		return;

	bool flag = false;

	if (new_num > num)
		new_num--;
	
	for (DF::FacesVector::iterator it_vf = vf_.begin(); it_vf != vf_.end();)
	{
		flag = false;
		for (DF::faceItemVector::iterator it_vf_face = it_vf->begin(); it_vf_face != it_vf->end(); ++it_vf_face)
		{
			if (it_vf_face->v == num)
			{
				if (b_vf)
				{
					it_vf = vf_.erase(it_vf);
					count_del_face++;
					flag = true;
					break;
				}
				else
				{
					it_vf_face->v = new_num;
					//it_vf_face->t = new_num;
				}
			}
			if (it_vf_face->v > num && it_vf_face->v != new_num)
			{
				it_vf_face->v--;
				if (tt) it_vf_face->t--;
			}
		}

		if (!flag) ++it_vf;
	}

	// œÓËÒÍ „ÛÔÔ˚ ÍÓÚÓÓÈ ÔËÌ‡‰ÎÂÊËÚ ‚Â¯ËÌ‡
	int num_group = -1;
	size_t i_group = 0;
	for (auto it_group = groups_.begin(); it_group != groups_.end(); ++it_group, ++i_group)
	{
		if (it_group->v_first  > num * 3)
			break;
		if (it_group->v_first <= num * 3 && it_group->v_last > num * 3)
			num_group = static_cast<int>(i_group);
	}

	if (num_group >= 0 && static_cast<size_t>(num_group) <= groups_.size())
	{
		groups_[num_group].v_last -= 3;
		groups_[num_group].f_last -= count_del_face;

		for (size_t i = static_cast<size_t>(num_group) + 1; i < groups_.size(); i++)
		{
			groups_[i].v_first -= 3;
			groups_[i].v_last -= 3;
			groups_[i].f_first -= count_del_face;
			groups_[i].f_last -= count_del_face;
		}
	}

	if (vn_.size() > 0)
		RecalcNormals();

	// ≈ÒÎË Ï‡ÚËˆ‡ ËÌ‰Ë‰ÂÌÚÌÓÒÚË ÌÂ ÔÛÒÚ‡, ÚÓ Á‡ÏÂÌˇÂÏ ‚ ÌÂÈ ÌÓÏÂ‡ ‚Â¯ËÌ ÚÓÊÂ
	DelIncedenceVector(num, new_num);
}

//
void DFMesh::DelIncedenceVector(size_t num_del_v, size_t num_new_v)
{
	if (incidence_.size() == 0)
		return;

	size_t num_del_v_3 = num_del_v * 3;
	size_t num_new_v_3 = num_new_v * 3;

	for (auto it_inced = incidence_.begin(); it_inced != incidence_.end(); ++it_inced)
	{
		for (auto it_record = it_inced->begin(); it_record != it_inced->end(); ++it_record)
		{
			if (it_record->index == num_del_v_3)
			{
				it_record->index = num_new_v_3;
				continue;
			}
			else if (it_record->index > num_del_v_3)
				it_record->index -= 3;
		}
	}

	// Á‰ÂÒ¸ num_new_v ‰ÓÎÊÂÌ ·˚Ú¸ Â˘Â ÒÚ‡˚Ï
	if (num_new_v >= num_del_v)
		num_new_v++;

	for (auto it_record = incidence_[num_del_v].begin(); it_record != incidence_[num_del_v].end(); ++it_record)
		incidence_[num_new_v].push_back(*it_record);

	incidence_.erase(incidence_.begin() + num_del_v);

    // Û‰‡ÎËÏ ÔÓ‚ÚÓ˚
    for (DF::IncedenceVector2D::iterator it = incidence_.begin(); it < incidence_.end(); it++)
    {
        std::sort( it->begin(), it->end() );
        it->erase( std::unique( it->begin(), it->end() ), it->end() );
    }
}

//
size_t DFMesh::CountRibsLongerValue(DF::FloatingPointType len_rib)
{
	size_t count_ribs = 0;

	for (auto it_inced = incidence_.begin(); it_inced != incidence_.end(); ++it_inced)
	{
		for (auto it = it_inced->begin(); it != it_inced->end(); ++it)
		{
			if (it->distance > len_rib)
				++count_ribs;
		}
	}

	return count_ribs;
}

//
DF::faceItemVector & DFMesh::GetFace(const size_t & num)
{
	return vf_[num];
}

//
void DFMesh::MoveZ(float z)
{
	if (vv_.size() < 3)
		return;
	
	for (size_t i = 2; i < vv_.size(); i += 3)
		vv_[i] += z;
}

//
void DFMesh::Move(float value, int os)
{
	if (vv_.size() < 3)
		return;

	size_t move_os = 0;
	switch(os)
	{
	case 2 : move_os = 1; break;
	case 3 : move_os = 2; break;
	case 1: // x
	default :
		move_os = 0;
		break;
	}
	
	for (size_t i = move_os; i < vv_.size(); i += 3)
		vv_[i] += value;
}

void DFMesh::RotateXYZ(const DFMath::Vector3 & angles)
{
	// ¬˚ÒÚ‡‚ÎˇÂÏ Ó·˙ÂÍÚ ‚ Á‡‰‡ÌÌÛ˛ ÔÓÁËˆË˛
	DFMath::Vector3 new_center((Max() + Min()) / 2);

	_TVector2d<DF::FloatingPointType> p_2d;
	for (size_t i = 0; i < CountVertex(); i+=3)
	{
		DFMath::Vector3 vec(GetVertex(i) - new_center.at(0),
							GetVertex(i + 1) - new_center.at(1),
							GetVertex(i + 2) - new_center.at(2));

		p_2d = _RotateVect(angles.at(0) / (DF::FloatingPointType)57.3, _TVector2d<DF::FloatingPointType>(vec.at(1), vec.at(2)));
		vec.set(vec.at(0), p_2d.XCoord, p_2d.YCoord);

		p_2d = _RotateVect(angles.at(1) / (DF::FloatingPointType)57.3, _TVector2d<DF::FloatingPointType>(vec.at(0), vec.at(2)));
		vec.set(p_2d.XCoord, vec.at(1), p_2d.YCoord);

		p_2d = _RotateVect(angles.at(2) / (DF::FloatingPointType)57.3, _TVector2d<DF::FloatingPointType>(vec.at(0), vec.at(1)));
		vec.set(p_2d.XCoord, p_2d.YCoord, vec.at(2));

		SetVertex(i, vec.at(0) + new_center.at(0));
		SetVertex(i + 1, vec.at(1) + new_center.at(1));
		SetVertex(i + 2, vec.at(2) + new_center.at(2));
	}
}

DF::Error_t DFMesh::Tighten1(DF::FloatingPointType stiffness_coefficient, size_t iteration_num/*, float step*/)
{
	if (incidence_.size() == 0)
		return DF::err_not_enough_elements;

    DF::FloatingPointType cur_length;
    DF::FloatingPointType spring_coeff;
    DFMath::Vector3 point1, point2, arc;
    std::vector<DFMath::Vector3> forces(vv_.size() / 3);
    std::vector<size_t> counter(vv_.size() / 3);
//     DF::FloatingPointType scalar_correction = 0;
    DFMath::Vector3 vect_correction(0.);

	DFMath::Vector3 temp_vector3(0.);

    for (size_t i1 = 0; i1 < iteration_num; i1++)
    {
        std::fill(forces.begin(), forces.end(), temp_vector3); 

        for (size_t i_cur_point = 0; i_cur_point < incidence_.size(); i_cur_point++)
        {
            DF::IncedenceVector &cur_incedence_vector = incidence_[i_cur_point];
            for (DF::IncedenceVector::iterator it_cur_linked = cur_incedence_vector.begin(); it_cur_linked < cur_incedence_vector.end(); it_cur_linked++)
            {
                point1.set(vv_[i_cur_point * 3], vv_[i_cur_point * 3 + 1], vv_[i_cur_point * 3 + 2]);
                point2.set(vv_[it_cur_linked->index], vv_[it_cur_linked->index + 1], vv_[it_cur_linked->index + 2]);

                arc.set(point2 - point1);
                cur_length = DFMath::length(arc);
                if (cur_length <1e-3/*= step*/)
                    continue;
                spring_coeff = (cur_length - it_cur_linked->distance * stiffness_coefficient) / cur_length;
                forces[i_cur_point] += arc * spring_coeff;
				// ≈ÒÎË ÚÓ˜Í‡ ¯‚‡, ÚÓ„‰‡ ÂÂ ÌÂ ÒÏÂ˘‡Ú¸ (Á‡ÍÂÔÎˇÂÚÒˇ ÊÂÒÚÍÓ)
				//if (seams[i_cur_point])
				//	continue;
                counter[i_cur_point]++;
            }
        }

        for (size_t i = 0; i < forces.size(); i++)
        {
            if (counter[i] == 0) continue;
            forces[i] /= static_cast<DF::FloatingPointType>(counter[i]);
            vv_[i * 3] += forces[i][0];
            vv_[i * 3 + 1] += forces[i][1];
            vv_[i * 3 + 2] += forces[i][2];
        }
    }

    return DF::err_ok;

    //if (incidence_.size() == 0)
    //    return DF::err_not_enough_elements;

    //DF::FloatingPointType cur_length;
    //DF::FloatingPointType spring_coeff;
    //DFMath::Vector3 point1, point2, arc;
    //std::vector<DFMath::Vector3> forces(vv_.size() / 3);
    //std::vector<size_t> counter(vv_.size() / 3);
    //DF::FloatingPointType scalar_correction = 0;
    //DFMath::Vector3 vect_correction(0.);

    //for (size_t i1 = 0; i1 < iteration_num; i1++)
    //{
    //    //for (size_t i2 = 0; i2 < 10; i2++)
    //    {
    //        // TODO: ÛÒÎÓ‚ËÂ Á‡‚Â¯ÂÌËˇ
    //        forces.resize(0);
    //        forces.resize(vv_.size() / 3, DFMath::Vector3(0.));

    //        for (size_t i_cur_point = 0; i_cur_point < incidence_.size(); i_cur_point++)
    //        {
    //            DF::IncedenceVector &cur_incedence_vector = incidence_[i_cur_point];
    //            for (DF::IncedenceVector::iterator it_cur_linked = cur_incedence_vector.begin(); it_cur_linked < cur_incedence_vector.end(); it_cur_linked++)
    //            {
    //                point1.set(vv_[i_cur_point * 3], vv_[i_cur_point * 3 + 1], vv_[i_cur_point * 3 + 2]);
    //                point2.set(vv_[it_cur_linked->index], vv_[it_cur_linked->index + 1], vv_[it_cur_linked->index + 2]);

    //                arc.set(point2 - point1);
    //                cur_length = DFMath::length(arc);
    //                if (cur_length < 1e-3)
    //                    continue;
    //                spring_coeff = (cur_length - it_cur_linked->distance * stiffness_coefficient) / cur_length;
    //                forces[i_cur_point] += arc * spring_coeff;
				//	// ≈ÒÎË ÚÓ˜Í‡ ¯‚‡, ÚÓ„‰‡ ÂÂ ÌÂ ÒÏÂ˘‡Ú¸ (Á‡ÍÂÔÎˇÂÚÒˇ ÊÂÒÚÍÓ)
				//	if (seams[i_cur_point])
				//		continue;
    //                counter[i_cur_point]++;
    //            }
    //        }

    //        for (size_t i = 0; i < forces.size(); i++)
    //        {
    //            if (counter[i] == 0) continue;
    //            forces[i] /= static_cast<DF::FloatingPointType>(counter[i]);
    //            vv_[i * 3] += forces[i][0];
    //            vv_[i * 3 + 1] += forces[i][1];
    //            vv_[i * 3 + 2] += forces[i][2];
    //        }
    //    }
    //}

    //return DF::err_ok;
}

DF::Error_t DFMesh::ColourInVertices(size_t start_point_index, DF::IndexVector &vv_colors, DF::Index &colors_num) const
{
    DF::Error_t err = DF::err_ok;
    vv_colors.resize(0);
    vv_colors.resize(vv_.size() / 3, 0);
    DF::IndexVector fixed_points(vv_.size() / 3, INT_MAX);
    size_t fixed_point_actual_size = 0;
    size_t fixed_points_size_last = 1;
    size_t fixed_points_size_curr = 0;
    DF::Index cur_color = 1;

    while (fixed_point_actual_size < fixed_points.size())
    {
        vv_colors[start_point_index] = cur_color;
        fixed_points[fixed_point_actual_size++] = start_point_index;
        DFMath::Vector3 point1(0.), point1n(0.), point2(0.), delta_vector(0.);
        DF::IndexVector layers;

        // ÔÓıÓ‰ ÔÓ „‡ÙÛ (‚ ¯ËËÌÛ)
        while ((fixed_point_actual_size < fixed_points.size() && fixed_points_size_last != fixed_points_size_curr))
        {
            // ÒÙÓÏËÛÂÏ ÒÎÓÈ
            fixed_points_size_last = fixed_points_size_curr;
            fixed_points_size_curr = fixed_point_actual_size;
            if (fixed_points_size_last != fixed_points_size_curr)
                layers.push_back(fixed_points_size_curr);
            for (size_t fixed_points_i = fixed_points_size_last; fixed_points_i < fixed_points_size_curr; fixed_points_i++)
            {
                DF::Index &cur_point_index = fixed_points[fixed_points_i];
                for (size_t incidence_i = 0; incidence_i < incidence_[cur_point_index].size(); incidence_i++)
                {
                    const DF::Index &cur_connected_point_index = incidence_[cur_point_index][incidence_i].index / 3;
                    if (vv_colors[cur_connected_point_index] == 0) {  // Â˘∏ ÌÂ ÔÓÈ‰ÂÌ‡
                        vv_colors[cur_connected_point_index] = cur_color;
                        fixed_points[fixed_point_actual_size++] = cur_connected_point_index;
                    }
                }
            }
            cur_color++;
        }
        start_point_index = find(vv_colors.begin(), vv_colors.end(), static_cast<DF::Index>(0)) - vv_colors.begin();
    }
    colors_num = cur_color;
    return err;
}

DF::Error_t DFMesh::ColourInFaces(const DF::IndexVector &vv_colors, DF::IndexVector &vf_colors) const
{
    DF::Error_t err = DF::err_ok;
    vf_colors.resize(0);
    vf_colors.resize(vf_.size(), 0);
    for (size_t face_i = 0; face_i < vf_.size(); face_i++)
    {
        // TODO: ˜ÚÓ·˚ ·˚ÎÓ "Í‡ÒË‚Ó" ÏÓÊÌÓ Í‡ÒËÚ¸ ÙÂÈÒ ‚ ÏËÌËÏ‡Î¸Ì˚È ˆ‚ÂÚ ÒÂ‰Ë ˆ‚ÂÚÓ‚ Â„Ó ‚Â¯ËÌ
        vf_colors[face_i] = vv_colors[vf_[face_i][0].v];
    }
    return err;
}

DF::Error_t DFMesh::ColourInVerticesFaces(const size_t start_point_index, DF::IndexVector &vv_colors, DF::IndexVector &vf_colors, DF::Index &colors_count) const
{
    DF::Error_t err = DF::err_ok;
    colors_count = 0;
    if((err = ColourInVertices(start_point_index, vv_colors, colors_count)) != DF::err_ok)
        return err;
    if((err = ColourInFaces(vv_colors, vf_colors)) != DF::err_ok)
        return err;
    return err;
}

DF::Error_t DFMesh::ReformMeshByGraphLevels(DF::IndexVector &vf_colors)
{
    DF::Error_t err = DF::err_ok;
    DF::Index colors_count = 0;
    DF::IndexVector vv_colors;
    if ((err = ColourInVerticesFaces(0, vv_colors, vf_colors, colors_count)) != DF::err_ok)
        return err;

    for (DF::Index cur_color = 0; cur_color < colors_count; cur_color++)
    {
        for (DF::IndexVector::iterator face_colors_iter = find(vf_colors.begin(), vf_colors.end(), cur_color); face_colors_iter < vv_colors.end(); face_colors_iter = find(++face_colors_iter, vf_colors.end(), cur_color)) ;
            
    }
    return err;
}

DF::Error_t DFMesh::ImportFile(const std::string &fn, FileType &file_type, bool bPack) {
    switch (file_type) {
    case FILETYPE_DFBINARY:
        if (ImportBIN(fn, bPack) == DF::err_ok) {
            return DF::err_ok;
        }

    case FILETYPE_WAVEFRONT:

    default:
        return ImportOBJ(fn, bPack);
    }
}

void DFMesh::MoveY(DF::FloatingPointType dy)
{
    if (vv_.size() < 3)
        return;

    for (size_t i = 1; i < vv_.size(); i += 3)
        vv_[i] += dy;
}

//
DFMath::Vector3 DFMesh::Min()
{
	if (vv_.size() < 3)
		return DFMath::Vector3();

	DFMath::Vector3 min = DFMath::Vector3(vv_[0], vv_[1], vv_[2]);
	for( size_t i = 0; i < vv_.size(); i+=3)
	{
		DFMath::Vector3 tmp_v(vv_[i], vv_[i+1], vv_[i+2]);

		if (tmp_v[0] < min[0]) min[0] = tmp_v[0];
		if (tmp_v[1] < min[1]) min[1] = tmp_v[1];
		if (tmp_v[2] < min[2]) min[2] = tmp_v[2];
	}

	return min;
}

//
DFMath::Vector3 DFMesh::Max()
{
	if (vv_.size() < 3)
		return DFMath::Vector3();

	DFMath::Vector3 max = DFMath::Vector3(vv_[0], vv_[1], vv_[2]);
	for( size_t i = 0; i < vv_.size(); i+=3)
	{
		DFMath::Vector3 tmp_v(vv_[i], vv_[i+1], vv_[i+2]);
		if (max[0] < tmp_v[0]) max[0] = tmp_v[0];
		if (max[1] < tmp_v[1]) max[1] = tmp_v[1];
		if (max[2] < tmp_v[2]) max[2] = tmp_v[2];
	}

	return max;
}

bool DFMesh::isExist() const
{
    return
        vv_.size() > 0 ||
        vf_.size() > 0 ||
        vn_.size() > 0 ||
        vt_.size() > 0;
}

DF::Error_t DFMesh::SmoothMesh(DFMesh::SmoothType smooth_method, const DF::FloatingPointType k)
{
    if (vv_.size() != vn_.size()) {
        log << '[' << DF::GetCurrentTimeString() << "] SmoothMesh(): " << DF::print_error(DF::err_not_enough_elements) << " (vv_.size()=" << vv_.size() << ", vn_.size()=" << vn_.size() << ")\n";
        return DF::err_not_enough_elements;
    }

    DF::PointsVector new_vv;
    if (smooth_method==DFMesh::SMOOTH_POST)
        new_vv = vv_;
    DF::PointsVector& new_vv_link = smooth_method==DFMesh::SMOOTH_POST ? new_vv : vv_;

    for (size_t i = 0, i_3 = 0; i_3 < vv_.size(); i++, i_3 += 3)
    {
        DFMath::Vector3 cur_point(vv_[i_3], vv_[i_3 + 1], vv_[i_3 + 2]);
        DFMath::Vector3 cur_normal(vn_[i_3], vn_[i_3 + 1], vn_[i_3 + 2]);
        DF::FloatingPointType movement = DF::zero;
        DF::IncedenceVector& cur_uncedence_vec = incidence_[i];
        if (cur_uncedence_vec.size() == 0)
            continue;
        for (DF::IncedenceVector::const_iterator cit_incedence = cur_uncedence_vec.begin(); cit_incedence != cur_uncedence_vec.end(); cit_incedence++)
        {
            // ‚ÓÁÏÓÊÌÓ, Ò‰ÂÎ‡Ú¸ ÔÓ‚ÂÍÛ Ì‡ ÚËÔ Â·‡ (Ó·Ò˜ËÚ˚‚‡Ú¸ ÚÓÎ¸ÍÓ ÒÚÛÍÚÛÌ˚Â)
            movement += DFMath::ScalarProduct(cur_normal, DFMath::Vector3(vv_[cit_incedence->index], vv_[cit_incedence->index + 1], vv_[cit_incedence->index + 2]) - cur_point);
        }
        movement *= k / cur_uncedence_vec.size();
        cur_normal *= movement;
        new_vv_link[i_3] += cur_normal[0];
        new_vv_link[i_3+1] += cur_normal[1];
        new_vv_link[i_3+2] += cur_normal[2];
    }

    vv_.swap(new_vv_link);

    return DF::err_ok;
}

size_t DFMesh::GetGroupIndex3(const size_t point_num_3)
{
    size_t ret = groups_.size();
    if (ret == 0) return 0;
    do ret--; while (ret > 0 && point_num_3 < groups_[ret].v_first);
    return ret;
}

bool DFMesh::Connected(DF::Index i, DF::Index j)
{
    bool ret = false;
    DF::Index ind3 = j * 3;
    const DF::IncedenceVector& iincedence = incidence_[i];
    for (DF::IncedenceVector::const_iterator cit = iincedence.cbegin(); cit != iincedence.cend(); cit++)
    {
        if (ind3 == cit->index) {
            ret = true;
            break;
        }
    }
    return ret;
}

bool DFMesh::Connected(DF::Index i, DF::Index j, DF::RibType rib_type)
{
    bool ret = false;
    DF::Index ind3 = j * 3;
    const DF::IncedenceVector& iincedence = incidence_[i];
    for (DF::IncedenceVector::const_iterator cit = iincedence.cbegin(); cit != iincedence.cend(); cit++)
    {
        if (ind3 == cit->index && rib_type == cit->type) {
            ret = true;
            break;
        }
    }
    return ret;
}

int DFMesh::GraphDistance(DF::Index i, DF::Index j)
{
    return 0;
}

void DFMesh::MergePoints(DF::Index i, DF::Index j, bool bMerge_normals_UVs)
{
    if (i == j)
        return;
    if (i > j)
        DF::swap2(i, j);

    DF::Index i3 = i * 3;
    DF::Index j3 = j * 3;

    for (DF::IndexVector4D::iterator component_x_it = vertex_allocation_map_.begin(); component_x_it < vertex_allocation_map_.end(); component_x_it++)
        for (DF::IndexVector3D::iterator component_y_it = component_x_it->begin(); component_y_it < component_x_it->end(); component_y_it++)
            for (DF::IndexVector2D::iterator component_z_it = component_y_it->begin(); component_z_it < component_y_it->end(); component_z_it++)
                for (DF::IndexVector::iterator idx_it = component_z_it->begin(); idx_it < component_z_it->end(); idx_it++)
                    if (*idx_it == j3)
                        component_z_it->erase(idx_it--);
                    else
                        if (*idx_it > j3)
                            *idx_it -= 3;

    bool bMergedVertex = false;
    bool bMergedTexture = false;
    bool bMergedNormal = false;

    if (vv_.size() >= i3 + 3) {
        vv_[i3] = (vv_[i3] + vv_[j3]) / 2;
        vv_[i3+1] = (vv_[i3+1] + vv_[j3+1]) / 2;
        vv_[i3+2] = (vv_[i3+2] + vv_[j3+2]) / 2;
        vv_.erase(vv_.begin() + j3, vv_.begin() + j3 + 3);
        bMergedVertex = true;
    }

    if (bMerge_normals_UVs)
    {
        if (vt_.size() >= i3 + 3 && vt_.size() == vv_.size()) {
            vt_[i3] = (vt_[i3] + vt_[j3]) / 2;
            vt_[i3+1] = (vt_[i3+1] + vt_[j3+1]) / 2;
            vt_[i3+2] = (vt_[i3+2] + vt_[j3+2]) / 2;
            vt_.erase(vt_.begin() + j3, vt_.begin() + j3 + 3);
            bMergedTexture = true;
        }
    }

    // TODO: ‚ ÒÎÛ˜‡Â Ì‡ÎË˜Ëˇ Û ÚÓ˜ÍË ÌÂÒÍÓÎ¸ÍËı ÚÂÍÒÚÛÌ˚ı ÍÓÓ‰ËÌ‡Ú ËÎË ÌÓÏ‡ÎÂÈ - ÓÌË ÓÒÚ‡ÌÛÚÒˇ ·ÂÁ ÒÒ˚ÎÓÍ Ì‡ ÌËı, ‰Ó‡·ÓÚ‡Ú¸
    if (bMerge_normals_UVs)
    {
        if (vn_.size() >= i3 + 3 && vn_.size() == vv_.size()) {
            vn_[i3] = (vn_[i3] + vn_[j3]) / 2;
            vn_[i3+1] = (vn_[i3+1] + vn_[j3+1]) / 2;
            vn_[i3+2] = (vn_[i3+2] + vn_[j3+2]) / 2;
            vn_.erase(vn_.begin() + j3, vn_.begin() + j3 + 3);
            bMergedNormal = true;
        }
    }

    std::vector<DF::Index> processed_UVs;
    std::vector<DF::Index> processed_normals;
    for (DF::FacesVector::iterator cur_face = vf_.begin(); cur_face != vf_.end(); cur_face++) {
        for (DF::faceItemVector::iterator cur_item = cur_face->begin(); cur_item != cur_face->end(); cur_item++) {
            if (cur_item->v == j) {
                cur_item->v = i;
                if (bMerge_normals_UVs) {
                    if (bMergedNormal && cur_item->n == j) {
                        cur_item->n = i;
                    } else {
                        processed_normals.push_back(cur_item->n);
                    }
                    if (bMergedTexture && cur_item->t == j) {
                        cur_item->t = i;
                    } else {
                        processed_UVs.push_back(cur_item->t);
                    }
                }
            }
            else
            if (cur_item->v > j) {
                cur_item->v--;
                if (bMergedNormal && cur_item->n > j) {
                    cur_item->n--;
                }
                if (bMergedTexture && cur_item->t > j) {
                    cur_item->t--;
                }
            }
        }
    }

    for (DF::GroupsVector::iterator cur_group = groups_.begin(); cur_group != groups_.end(); cur_group++) {
        if (cur_group->v_last > j3) {
            if (bMergedVertex) cur_group->v_last -= 3;
            if (bMergedTexture) cur_group->t_last -= 3;
            if (bMergedNormal) cur_group->n_last -= 3;
            for (DF::GroupsVector::iterator next_group = cur_group + 1; next_group < groups_.end(); next_group++) {
                if (bMergedVertex) { 
                    next_group->v_first -= 3;
                    next_group->v_last -= 3;
                }
                if (bMergedTexture) {
                    next_group->t_first -= 3;
                    next_group->t_last -= 3;
                }
                if (bMergedNormal) {
                    next_group->n_first -= 3;
                    next_group->n_last -= 3;
                }
            }
            break;
        }
    }
}

DF::Error_t DFMesh::MergeCoincidentVertices()
{
    size_t counter = 0;
    for (size_t point_index = 0; point_index < vv_.size() / 3; point_index++)
    {
        DFMath::Vector3 specified_point(vv_[point_index * 3], vv_[point_index * 3 + 1], vv_[point_index * 3 + 2]);
        DF::Index nearest_point_index = 0;
        size_t i = 0;
        while (FindIdentical(nearest_point_index, point_index) == DF::err_ok && i++ < 100) {
            MergePoints(point_index, nearest_point_index);
            if (point_index != nearest_point_index)
                counter++;
        }
    }
    log << "[" << DF::GetCurrentTimeString() << "] " << counter << " points have been merged\n";
    return DF::err_ok;
}

DF::Error_t DFMesh::FindIdentical(DF::Index &identical_point_index, const DF::Index& specified_point_index) const
{
    identical_point_index = static_cast<DF::Index>(-1);
    const DF::PointsVector &vvLink = vv_;
    DF::Index specified_point_index_3 = specified_point_index * 3;
    DFMath::Vector3 specified_point(vvLink[specified_point_index_3], vvLink[specified_point_index_3+1], vvLink[specified_point_index_3+2]);
    int cube_index_x = static_cast<int>((specified_point[0] + cube_origin_shift_x) / cube_section_size_x);
    int cube_index_x_begin = cube_index_x;
    int cube_index_x_end = cube_index_x;
    int cube_index_y = static_cast<int>((specified_point[1] + cube_origin_shift_y) / cube_section_size_y);
    int cube_index_y_begin = cube_index_y;
    int cube_index_y_end = cube_index_y;
    int cube_index_z = static_cast<int>((specified_point[2] + cube_origin_shift_z) / cube_section_size_z);
    int cube_index_z_begin = cube_index_z;
    int cube_index_z_end = cube_index_z;
    size_t counter = 1;

    do
    {
        int szl = static_cast<int>(vertex_allocation_map_.size());
        for (int ix = cube_index_x_begin; ix <= cube_index_x_end; ix++)
        {
            if (ix >= szl)
                break;
            DF::IndexVector3D &vertex_allocation_map_level1 = vertex_allocation_map_[ix];
            int szl1 = static_cast<int>(vertex_allocation_map_level1.size());
            for (int iy = cube_index_y_begin; iy <= cube_index_y_end; iy++)
            {
                if (iy >= szl1)
                    break;
                DF::IndexVector2D &vertex_allocation_map_level2 = vertex_allocation_map_level1[iy];
                int szl2 = static_cast<int>(vertex_allocation_map_level2.size());
                for (int iz = cube_index_z_begin; iz <= cube_index_z_end; iz++)
                {
                    if (iz >= szl2)
                        break;
                    DF::IndexVector &vertex_allocation_map_level3 = vertex_allocation_map_level2[iz];
                    DF::IndexVector::const_iterator endl3 = vertex_allocation_map_level3.end();
                    for (DF::IndexVector::const_iterator iti = vertex_allocation_map_level3.begin(); iti < endl3; iti++)
                    {
                        DF::Index ind = *iti;
                        if (DFMath::TaxicabLength(DFMath::Vector3(vvLink[ind], vvLink[ind+1], vvLink[ind+2]) - specified_point) <= 1e-3 && ind != specified_point_index_3)
                        {
                            identical_point_index = ind / 3;
                            goto break_loops;
                        }
                    }
                }
            }
        }
        counter++;

        if (counter <= 1)
        {
            cube_index_x_begin--;
            if (cube_index_x_begin < 0)
                break;
            cube_index_y_begin--;
            if (cube_index_y_begin < 0)
                break;
            cube_index_z_begin--;
            if (cube_index_z_begin < 0)
                break;
            cube_index_x_end++;
            cube_index_y_end++;
            cube_index_z_end++;
        }
    } while (identical_point_index == static_cast<DF::Index>(-1) && counter <= 1);

break_loops:
    return identical_point_index == static_cast<DF::Index>(-1) ? DF::err_cant_find_nearest : DF::err_ok;
}
