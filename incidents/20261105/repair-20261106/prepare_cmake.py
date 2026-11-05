from pathlib import Path
ROOT=Path(__file__).resolve().parent
p=ROOT/'staged/CMakeLists.txt'
text=(ROOT/'baseline/CMakeLists.txt').read_text(encoding='utf-8')
needle='        add_test(NAME px4_vio_regression COMMAND px4_vio_regression ${PROJECT_SOURCE_DIR})'
assert text.count(needle)==1
text=text.replace(needle,needle+'''
        add_executable(px4_recording_gate_regression tests/px4_recording_gate_regression.cc)
        target_link_libraries(px4_recording_gate_regression orbslam3_px4_core)
        set_target_properties(px4_recording_gate_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
        add_test(NAME px4_recording_gate_regression COMMAND px4_recording_gate_regression
            ${PROJECT_SOURCE_DIR}/config/px4_vio_local.yaml)''')
p.write_text(text,encoding='utf-8',newline='\n')
(ROOT/'staged/tests/px4_recording_gate_regression.cc').write_bytes((ROOT/'gate_recording_regression.cc').read_bytes())
print('STAGED_CMAKE_REGRESSION')
