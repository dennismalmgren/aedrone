
import freetype
face = freetype.Face(r'C:\Windows\Fonts\arial.ttf')
face.set_char_size( 48*64 )
face.load_char('S')
bitmap = face.glyph.bitmap
print(bitmap.buffer)
