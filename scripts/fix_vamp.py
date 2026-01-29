import re

# Change this file path to the actual path of 'frida_real.hh' on your system
file_path = "/home/dominguez/roborregos/home_ws/src/manipulation/packages/vamp/src/impl/vamp/robots/frida_real.hh"

with open(file_path, 'r') as f:
    content = f.read()

fixed_content = re.sub(r'{\s*(?://.*)?\s*}', '{\n      return false;\n    }', content)

with open(file_path, 'w') as f:
    f.write(fixed_content)

print("'return false;' inyectado en todos los bloques de colisión.")