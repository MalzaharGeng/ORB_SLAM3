"""Prepare the smallest gate fix on a copy of the authoritative Nano baseline."""
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parent
def replace(path, old, new):
    p=ROOT/'staged'/path
    text=p.read_text(encoding='utf-8')
    assert text.count(old)==1,(path,old)
    p.write_text(text.replace(old,new),encoding='utf-8',newline='\n')

def main():
    for path in ['src/Px4Vio.cc','include/Px4Vio.h','src/Px4Bridge.cc','config/px4_vio_local.yaml','config/px4_vio.yaml','tests/px4_vio_regression.cc']:
        target=ROOT/'staged'/path;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/'baseline'/path,target)
    replace('include/Px4Vio.h','    int domainId = 0;',
            '    // DDS jitter limits; freshness and timestamp age are independent gates.\n'
            '    double maxSyncRttSeconds = .05, maxSyncResidualSeconds = .01, maxSyncAgeSeconds = 2.;\n'
            '    int domainId = 0;')
    replace('src/Px4Vio.cc','    const double domain=number(fs,"domain_id",0);',
            '    c.maxSyncRttSeconds=number(fs,"max_sync_rtt_seconds",c.maxSyncRttSeconds);\n'
            '    c.maxSyncResidualSeconds=number(fs,"max_sync_residual_seconds",c.maxSyncResidualSeconds);\n'
            '    c.maxSyncAgeSeconds=number(fs,"max_sync_age_seconds",c.maxSyncAgeSeconds);\n'
            '    const double domain=number(fs,"domain_id",0);')
    replace('src/Px4Vio.cc','maxSampleGap>1)','maxSampleGap>2)')
    replace('src/Px4Vio.cc','    if(domainId<0 || domainId>232)throw',
            '    if(!std::isfinite(maxSyncRttSeconds) || maxSyncRttSeconds<=0 || maxSyncRttSeconds>.1 ||\n'
            '       !std::isfinite(maxSyncResidualSeconds) || maxSyncResidualSeconds<=0 || maxSyncResidualSeconds>.05 ||\n'
            '       !std::isfinite(maxSyncAgeSeconds) || maxSyncAgeSeconds<=0 || maxSyncAgeSeconds>2)\n'
            '        throw std::runtime_error("Invalid PX4 time-sync quality limits");\n'
            '    if(domainId<0 || domainId>232)throw')
    replace('src/Px4Vio.cc','    if(!t.haveSync || !fresh(steady,t.syncSteadyUs,2000000) || t.roundTripUs>20000 ||\n       std::abs(double(t.observedOffsetUs)-double(t.estimatedOffsetUs))>5000)return fail("time_sync_unready");',
            '    if(!t.haveSync || !fresh(steady,t.syncSteadyUs,uint64_t(config_.maxSyncAgeSeconds*1e6)) ||\n'
            '       t.roundTripUs>config_.maxSyncRttSeconds*1e6 ||\n'
            '       std::abs(double(t.observedOffsetUs)-double(t.estimatedOffsetUs))>config_.maxSyncResidualSeconds*1e6)\n'
            '        return fail("time_sync_unready");')
    replace('config/px4_vio_local.yaml','max_sample_gap_seconds: 0.30',
            '# Tm7tjC: valid observations separated by 1.067747s after DDS jitter.\n'
            '# Motion consistency checks still apply across the entire interval.\n'
            'max_sample_gap_seconds: 1.5\n'
            '# Recorded maxima: RTT 23.794ms; absolute offset residual 6.378ms.\n'
            'max_sync_rtt_seconds: 0.05\nmax_sync_residual_seconds: 0.01\nmax_sync_age_seconds: 2.0')
    replace('src/Px4Bridge.cc','            double hz=0;std::string lastReason;',
            '            double hz=0;std::string lastReason;\n'
            '            uint64_t noOutputSince=0,lastWarning=0;')
    replace('src/Px4Bridge.cc','                if(output) {\n                    const auto& o=*output;Odom msg{};',
            '                if(output || !publishedOnce) {noOutputSince=0;lastWarning=0;}\n'
            '                else {\n'
            '                    if(!noOutputSince)noOutputSince=steady;\n'
            '                    if(steady-noOutputSince>=10000000 && (!lastWarning || steady-lastWarning>=10000000)) {\n'
            '                        std::cerr<<"[PX4] WARN no accepted observation for "<<(steady-noOutputSince)*1e-6\n'
            '                            <<" s; reason="<<gate.Reason()<<" armed="<<telemetry.armed<<std::endl;\n'
            '                        lastWarning=steady;\n'
            '                    }\n'
            '                }\n'
            '                if(output) {\n                    const auto& o=*output;Odom msg{};')
    print('STAGED_CORE_FIX')

if __name__=='__main__':main()
