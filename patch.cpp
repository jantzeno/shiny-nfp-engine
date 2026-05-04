            auto pb = geom::compute_bounds(p.polygon);
            SHINY_DEBUG("solve.cpp: piece_id={} bounds x=[{}, {}] y=[{}, {}]", i.piece_id, pb.min.x(), pb.max.x(), pb.min.y(), pb.max.y());
            auto bin = std::find_if(request.bins.begin(), request.bins.end(), [&](const auto& b){ return b.bin_id == p.placement.bin_id; });
            auto bb = geom::compute_bounds(bin->polygon);
            SHINY_DEBUG("solve.cpp: bin_id={} bounds x=[{}, {}] y=[{}, {}]", bin->bin_id, bb.min.x(), bb.max.x(), bb.min.y(), bb.max.y());
